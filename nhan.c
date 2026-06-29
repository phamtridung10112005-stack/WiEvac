/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>         // Thêm thư viện toán học để tính toán DSP
#include "esp_timer.h"    // Lấy mốc thời gian chạy hệ thống để làm mỏ neo AI
#include "lwip/sockets.h" // Thêm thư viện mạng để chạy UDP Sockets
#include "esp_event.h"    // Quản lý sự kiện kết nối mạng

#include "nvs_flash.h"
#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_csi_gain_ctrl.h"

// ĐỒNG BỘ KÊNH 11 THEO ĐÚNG KÊNH PHÁT THỰC TẾ CỦA PI 5 VÀ C6
#define CONFIG_LESS_INTERFERENCE_CHANNEL   11

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0))
#define CONFIG_WIFI_BAND_MODE               WIFI_BAND_MODE_2G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT20  // ĐỒNG BỘ: Hạ về băng thông 20MHz
#define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT20  // ĐỒNG BỘ: Hạ về băng thông 20MHz
#define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
#define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#else
#define CONFIG_WIFI_BANDWIDTH           WIFI_BW_HT20      // ĐỒNG BỘ: Hạ băng thông lõi S3 về 20MHz
#endif

#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT20   // ĐỒNG BỘ: Chuyển PHY Mode ESP-NOW về HT20
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

// CẤU HÌNH ĐƯỜNG TRUYỀN UDP HƯỚNG VỀ BỘ NÃO PI 5
#define PI_GATEWAY_IP   "10.42.0.1"    // IP mạng Hotspot Pi 5
#define PI_UDP_PORT     8888           // Cổng mạng hứng dữ liệu trên Pi 5
#define PI_WIFI_SSID    "WiEvac_Pi5"   // Tên Wi-Fi của Pi 5
#define PI_WIFI_PASS    "12345678"     // Mật khẩu Wi-Fi của Pi 5

// static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";

static int g_udp_socket = -1;
static struct sockaddr_in g_pi_addr;

// ĐỊNH NGHĨA STRUCT ĐẶC TRƯNG CHUẨN 13 BYTES
typedef struct __attribute__((packed)) {
    uint8_t node_id;       // 1 Byte
    float std_dev;         // 4 Bytes
    float mean_amp;        // 4 Bytes
    uint32_t timestamp;    // 4 Bytes
} csi_feature_packet_t;

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

// Bộ quản lý sự kiện mạng: Chỉ kích hoạt bộ lọc sóng khi kết nối mạng hoàn tất ổn định
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "🎉 BẮT TAY PI 5 THÀNH CÔNG! Kích hoạt chế độ đón sóng CSI...");
        esp_wifi_set_promiscuous(true); // Kích hoạt Sniffer tầng vật lý công khai
        esp_wifi_set_csi(true);        // Mở van trích xuất ma trận CSI
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Bị ngắt kết nối! Đang tạm tắt bộ lọc để bám lại mạng Pi 5...");
        esp_wifi_set_promiscuous(false);
        esp_wifi_set_csi(false);
        esp_wifi_connect();
    }
}

static void udp_client_init(void) {
    g_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_udp_socket < 0) {
        ESP_LOGE(TAG, "Lỗi tạo Socket UDP!");
        return;
    }
    memset(&g_pi_addr, 0, sizeof(g_pi_addr));
    g_pi_addr.sin_addr.s_addr = inet_addr(PI_GATEWAY_IP);
    g_pi_addr.sin_family = AF_INET;
    g_pi_addr.sin_port = htons(PI_UDP_PORT);
    ESP_LOGI(TAG, "Mở Socket hướng về bộ não Pi 5 [%s:%d] thành công!", PI_GATEWAY_IP, PI_UDP_PORT);
}

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Đăng ký Event để theo dõi cái bắt tay của Pi 5
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = PI_WIFI_SSID,
            .password = PI_WIFI_PASS,
            .channel = 0, // Để bằng 0 để chip tự do quét và đồng bộ cấu hình Kênh/Băng thông của Pi 5
            .pmf_cfg = {
                .capable = true,          
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    // CẤU HÌNH BĂNG THÔNG SAU KHI START ĐỂ TRÁNH LỖI CRASH
#if !CONFIG_IDF_TARGET_ESP32C5 && !(CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)) && !CONFIG_IDF_TARGET_ESP32C61
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, CONFIG_WIFI_BANDWIDTH));
#endif

    // Cấu hình IP Tĩnh 10.42.0.2 chạy ngầm ngay sau khi start
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif)); 
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 10, 42, 0, 2);       
    IP4_ADDR(&ip_info.gw, 10, 42, 0, 1);       
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));

    ESP_LOGI(TAG, "Đường truyền IP Tĩnh đã sẵn sàng. Đang tiến hành kết nối...");
    esp_wifi_connect();
}

static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_LOGI(TAG, "Khởi tạo mạng ESP-NOW lắng nghe kênh CSI sạch sẽ.");
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;

    // VÔ HIỆU HÓA BỘ LỌC MAC: Chấp nhận tất cả gói tin thu được trên Kênh 11 để chống lệch danh tính MAC Broadcast
    // if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) return;

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

    int num_subcarriers = info->len / 2; 
    float amp_sum = 0;
    int valid_subcarriers_count = 0;

    // Vòng lặp DSP duyệt qua dải Subcarriers của băng thông 20MHz (Khoảng 52-56 sóng con hợp lệ)
    for (int i = 0; i < num_subcarriers; i++) {
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

    static float ema_amp = -1.0f;
    const float alpha = 0.08f;
    if (ema_amp < 0) ema_amp = current_packet_amp;
    else ema_amp = (alpha * current_packet_amp) + ((1.0f - alpha) * ema_amp);

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

        // ĐÓNG GÓI VECTOR ĐẶC TRƯNG TINH KHIẾT 13 BYTES
        csi_feature_packet_t packet;
        packet.node_id = 1; 
        packet.std_dev = std_dev;
        packet.mean_amp = mean;
        packet.timestamp = (uint32_t)(esp_timer_get_time() / 1000); 

        printf("[COMPRESSED] Size: %d Bytes | Node: %d | STD: %.4f | Mean: %.4f | TS: %lu\n", 
               (int)sizeof(packet), packet.node_id, packet.std_dev, packet.mean_amp, (unsigned long)packet.timestamp);

        // NÃ UDP THẲNG LÊN IP CARD MẠNG HOTSPOT CỦA PI 5
        if (g_udp_socket >= 0) {
            sendto(g_udp_socket, &packet, sizeof(packet), 0, (struct sockaddr *)&g_pi_addr, sizeof(g_pi_addr));
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
        .acquire_csi_ht40         = false, // ĐỒNG BỘ: Tắt trích xuất HT40
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
        .acquire_csi_ht40       = false, // ĐỒNG BỘ: Tắt trích xuất HT40
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
