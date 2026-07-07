/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>         // DSP math
#include "esp_timer.h"    // Timestamp
#include "lwip/sockets.h" // UDP Sockets
#include "esp_event.h"    // Wi-Fi events

#include "nvs_flash.h"
#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_csi_gain_ctrl.h"

// --- [THÊM MỚI] THƯ VIỆN & CẤU HÌNH ---
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FAIL_SAFE_PIN 4             // Chân GPIO còi hú/đèn LED
#define STD_DEV_THRESHOLD 1.5       // Ngưỡng tĩnh báo động khi mất Pi 5

uint8_t g_auto_node_id = 0;         
bool g_is_standalone_mode = true;   // Khởi động mặc định là chế độ sinh tồn (chưa có mạng)
// ---------------------------------------------------

// ĐỒNG BỘ KÊNH 11
#define CONFIG_LESS_INTERFERENCE_CHANNEL   11

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0))
#define CONFIG_WIFI_BAND_MODE               WIFI_BAND_MODE_2G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT20  
#define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT20  
#define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
#define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#else
#define CONFIG_WIFI_BANDWIDTH           WIFI_BW_HT20      
#endif

#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT20   
#define CONFIG_ESP_NOW_RATE             WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_FORCE_GAIN                   0

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF                      0
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define CONFIG_GAIN_CONTROL                 1
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

// CẤU HÌNH PI 5
#define PI_GATEWAY_IP   "10.42.0.1"    
#define PI_UDP_PORT     8888           
#define PI_WIFI_SSID    "WiEvac_Pi5"   
#define PI_WIFI_PASS    "12345678"     

static const char *TAG = "csi_recv";

static int g_udp_socket = -1;
static struct sockaddr_in g_pi_addr;

// STRUCT 13 BYTES
typedef struct __attribute__((packed)) {
    uint8_t node_id;       
    float std_dev;         
    float mean_amp;        
    uint32_t timestamp;    
} csi_feature_packet_t;

// Hàm hỗ trợ Quick Sort cho bộ lọc Hampel
static void quick_sort(float arr[], int left, int right) {
    int i = left, j = right;
    float tmp;
    float pivot = arr[(left + right) / 2];
    while (i <= j) {
        while (arr[i] < pivot) i++;
        while (arr[j] > pivot) j--;
        if (i <= j) {
            tmp = arr[i]; arr[i] = arr[j]; arr[j] = tmp;
            i++; j--;
        }
    }
    if (left < j) quick_sort(arr, left, j);
    if (i < right) quick_sort(arr, i, right);
}

// ==============================================================================
// BỘ QUẢN LÝ SỰ KIỆN WI-FI (ĐÃ TỐI ƯU CƠ CHẾ AUTO-RECOVERY LÌ LỢM)
// ==============================================================================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // Bắt đầu vòng đời kết nối
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "🎉 KẾT NỐI WI-FI THÀNH CÔNG! Đang chờ DHCP...");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Đã nhận IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // CÓ MẠNG: Tắt sinh tồn, tắt còi
        g_is_standalone_mode = false;
        gpio_set_level(FAIL_SAFE_PIN, 0);

        // Bật đón sóng CSI
        esp_wifi_set_promiscuous(true); 
        esp_wifi_set_csi(true);
        ESP_LOGI(TAG, "Kích hoạt CSI. Đang bắn data lên Pi 5!");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Tắt đón sóng CSI để dồn tài nguyên reconnect
        esp_wifi_set_promiscuous(false);
        esp_wifi_set_csi(false);
        
        // MẤT MẠNG: Bật sinh tồn, chuẩn bị hú còi nếu có biến
        g_is_standalone_mode = true;
        ESP_LOGW(TAG, "⚠️ MẤT PI 5! Kích hoạt sinh tồn. Đang thử kết nối lại...");
        
        // Ép mạch liên tục gọi cửa kết nối lại (Bỏ luôn giới hạn đếm số lần)
        esp_wifi_connect();
    }
}

static void udp_client_init(void) {
    g_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_udp_socket < 0) {
        ESP_LOGE(TAG, "Lỗi tạo UDP Socket!");
        return;
    }
    memset(&g_pi_addr, 0, sizeof(g_pi_addr));
    g_pi_addr.sin_addr.s_addr = inet_addr(PI_GATEWAY_IP);
    g_pi_addr.sin_family = AF_INET;
    g_pi_addr.sin_port = htons(PI_UDP_PORT);
}

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Đăng ký Event Loop
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = PI_WIFI_SSID,
            .password = PI_WIFI_PASS,
            .channel = 0, 
            .pmf_cfg = {
                .capable = true,          
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // CẤM NGỦ ĐỂ BẮT ĐỦ GÓI ESP-NOW
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if !CONFIG_IDF_TARGET_ESP32C5 && !(CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)) && !CONFIG_IDF_TARGET_ESP32C61
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, CONFIG_WIFI_BANDWIDTH));
#endif
}

static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// ==============================================================================
// XỬ LÝ TÍN HIỆU SỐ (DSP) & ĐÓNG GÓI
// ==============================================================================
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    static int s_count = 0;
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;

#if CONFIG_GAIN_CONTROL
    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < 100) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    } else if (s_count == 100) {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
#if CONFIG_FORCE_GAIN
        esp_csi_gain_ctrl_set_rx_force_gain(agc_gain_baseline, fft_gain_baseline);
#endif
    }
    esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
#endif

    // Rút trích Biên độ
    int num_subcarriers = info->len / 2; 
    float amp_sum = 0;
    int valid_subcarriers_count = 0;

    for (int i = 0; i < num_subcarriers; i++) {
        // Lọc bỏ kênh hoa tiêu rác
        if (i < 6 || i > (num_subcarriers - 6) || (i > (num_subcarriers / 2 - 3) && i < (num_subcarriers / 2 + 3))) {
            continue;
        }
        int8_t i_real = info->buf[2 * i];
        int8_t q_imag = info->buf[2 * i + 1];
        float amplitude = sqrtf((float)(i_real * i_real + q_imag * q_imag)) * compensate_gain;
        amp_sum += amplitude;
        valid_subcarriers_count++;
    }

    if (valid_subcarriers_count == 0) return;
    float current_packet_amp = amp_sum / valid_subcarriers_count;

    // Lọc Hampel gạt Outlier
    static float hampel_window[5] = {0};
    static int hampel_idx = 0;
    hampel_window[hampel_idx] = current_packet_amp;
    hampel_idx = (hampel_idx + 1) % 5;

    float sorted_window[5];
    memcpy(sorted_window, hampel_window, sizeof(hampel_window));
    quick_sort(sorted_window, 0, 4);
    float median = sorted_window[2];

    float abs_diff[5];
    for(int i=0; i<5; i++) abs_diff[i] = fabsf(hampel_window[i] - median);
    quick_sort(abs_diff, 0, 4);
    float mad = abs_diff[2];

    if (fabsf(current_packet_amp - median) > (3.0f * 1.4826f * mad) && mad > 0.001f) {
        current_packet_amp = median;
    }

    // Lọc Trung bình trượt hàm mũ (EMA) làm mượt sóng
    static float ema_amp = -1.0f;
    const float alpha = 0.08f;
    if (ema_amp < 0) ema_amp = current_packet_amp;
    else ema_amp = (alpha * current_packet_amp) + ((1.0f - alpha) * ema_amp);

    // Gom mẫu tính Tần suất & Rung lắc (100 packets/block)
    static float feature_window[100] = {0};
    static int feature_count = 0;
    feature_window[feature_count++] = ema_amp;

    if (feature_count >= 100) {
        float sum = 0;
        for (int i = 0; i < 100; i++) sum += feature_window[i];
        float mean = sum / 100.0f; 

        float variance_sum = 0;
        for (int i = 0; i < 100; i++) variance_sum += (feature_window[i] - mean) * (feature_window[i] - mean);
        float std_dev = sqrtf(variance_sum / 100.0f); 

        csi_feature_packet_t packet;
        packet.node_id = g_auto_node_id;  
        packet.std_dev = std_dev;
        packet.mean_amp = mean;
        packet.timestamp = (uint32_t)(esp_timer_get_time() / 1000); 

        // LOGIC ĐIỀU PHỐI ĐẦU RA 
        if (g_is_standalone_mode) {
            // Mất Pi 5 -> Dùng luật If/Else nội bộ để hú còi 
            // Nếu độ ồn vượt 2 lần ngưỡng tĩnh -> Có nguy hiểm
            if (std_dev > (STD_DEV_THRESHOLD * 2.0)) {
                gpio_set_level(FAIL_SAFE_PIN, 1);
            } else {
                gpio_set_level(FAIL_SAFE_PIN, 0);
            }
        } else {
            // Có Pi 5 -> Chăm chỉ nhả data 1 lần/giây
            if (g_udp_socket >= 0) {
                sendto(g_udp_socket, &packet, sizeof(packet), 0, (struct sockaddr *)&g_pi_addr, sizeof(g_pi_addr));
            }
        }
        
        feature_count = 0;
    }
    s_count++;
}

static void wifi_csi_init()
{
    #if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = false,
        .acquire_csi_force_lltf   = CSI_FORCE_LLTF,
        .acquire_csi_ht20         = true,
        .acquire_csi_ht40         = false, 
        .acquire_csi_vht          = false,
        .acquire_csi_su           = false,
        .acquire_csi_mu           = false,
        .acquire_csi_dcm          = false,
        .acquire_csi_beamformed   = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg            = 0,
        .dump_ack_en              = false,
        .reserved                 = false
    };
    #elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,
        .acquire_csi_legacy     = false,
        .acquire_csi_ht20       = true,
        .acquire_csi_ht40       = false, 
        .acquire_csi_su         = true,
        .acquire_csi_mu         = true,
        .acquire_csi_dcm        = true,
        .acquire_csi_beamformed = true,
        .acquire_csi_he_stbc    = 2,
        .val_scale_cfg          = false,
        .dump_ack_en            = false,
        .reserved               = false
    };
    #else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
    #endif
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Cấu hình chân còi hú/đèn LED (FAIL_SAFE_PIN)
    gpio_reset_pin(FAIL_SAFE_PIN);
    gpio_set_direction(FAIL_SAFE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FAIL_SAFE_PIN, 0);

    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
    g_auto_node_id = mac_addr[5]; 
    
    ESP_LOGI("MAIN", "==================================");
    ESP_LOGI("MAIN", "🚀 WIEVAC EDGE NODE (ID: %d) READY!", g_auto_node_id);
    ESP_LOGI("MAIN", "==================================");

    wifi_init();

    esp_now_peer_info_t peer = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };

    wifi_esp_now_init(peer);
    wifi_csi_init();
    udp_client_init(); 
}
