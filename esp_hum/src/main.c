#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_gap_ble_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "math.h"
#include "driver/gpio.h"

static const char *TAG = "MOBILE";

// --- Конфигурация сети и сервера ---
#define WIFI_SSID "ESPSERVER"
#define WIFI_PASS "ESPSERVER"
#define SERVER_URL "http://192.168.4.1" // Базовый URL сервера

// --- Конфигурация BLE и позиционирования ---
#define BEACON1_MAJOR 0x0001
#define BEACON2_MAJOR 0x0002
#define BEACON3_MAJOR 0x0003  // Новый маяк (сервер)
#define COMMON_UUID "FDA50693A4E24FB1AFCFC6EB07647825"
#define BEACON_DIST_M 2.0 // Расстояние между маяками 1 и 2 в метрах
#define BEACON3_X 1.0     // Позиция Beacon3 (сервер): предполагаем треугольник
#define BEACON3_Y 1.732   // √3 для равностороннего треугольника ~2м
#define PATH_LOSS_N 3.0   // Уменьшение погрешности: стандартное для indoor (было 2.5)
#define RSSI_THRESHOLD -85 // Уменьшение чувствительности: игнорируем слабее (было -90)
#define BUZZER_PIN GPIO_NUM_2 // Пин для подключения зуммера
#define RSSI_AVG_WINDOW 5 // Размер окна для moving average RSSI

// Глобальные переменные для Wi-Fi и данных с маяков
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

typedef struct {
    int rssi1, rssi2, rssi3;
    TickType_t last_seen1, last_seen2, last_seen3;
    int rssi_history1[RSSI_AVG_WINDOW], rssi_history2[RSSI_AVG_WINDOW], rssi_history3[RSSI_AVG_WINDOW];
    int rssi_count1, rssi_count2, rssi_count3;
} beacon_data_t;

// Инициализируем со значениями, указывающими на отсутствие сигнала
static beacon_data_t beacons = { 
    .rssi1 = -100, .rssi2 = -100, .rssi3 = -100, 
    .last_seen1 = 0, .last_seen2 = 0, .last_seen3 = 0,
    .rssi_count1 = 0, .rssi_count2 = 0, .rssi_count3 = 0
};

// Функция для moving average RSSI
static int average_rssi(int *history, int count, int new_rssi) {
    if (count < RSSI_AVG_WINDOW) {
        history[count++] = new_rssi;
    } else {
        for (int i = 0; i < RSSI_AVG_WINDOW - 1; i++) {
            history[i] = history[i + 1];
        }
        history[RSSI_AVG_WINDOW - 1] = new_rssi;
    }
    int sum = 0;
    for (int i = 0; i < (count < RSSI_AVG_WINDOW ? count : RSSI_AVG_WINDOW); i++) {
        sum += history[i];
    }
    return sum / (count < RSSI_AVG_WINDOW ? count : RSSI_AVG_WINDOW);
}

// Функция расчёта расстояния по RSSI
static double calculate_distance(int rssi, int tx_power) {
    if (rssi == 0) return -1.0;
    double distance = pow(10.0, ((double)tx_power - rssi) / (10.0 * PATH_LOSS_N));
    return distance;
}

// Функция трилатерации для 3 маяков (least-squares approximation)
static void trilaterate_position(double *x, double *y, double d1, double d2, double d3) {
    // Beacon1 (0,0), Beacon2 (BEACON_DIST_M, 0), Beacon3 (BEACON3_X, BEACON3_Y)
    double A = 2 * BEACON_DIST_M - 2 * 0;
    double B = 2 * 0 - 2 * 0;
    double C = pow(d1, 2) - pow(d2, 2) + pow(BEACON_DIST_M, 2) - pow(0, 2) + pow(0, 2) - pow(0, 2);
    double D = 2 * BEACON3_X - 2 * BEACON_DIST_M;
    double E = 2 * BEACON3_Y - 2 * 0;
    double F = pow(d2, 2) - pow(d3, 2) + pow(BEACON3_X, 2) - pow(BEACON_DIST_M, 2) + pow(BEACON3_Y, 2) - pow(0, 2);
    
    double denom = A * E - B * D;
    if (fabs(denom) < 1e-6) { // Коллинеарны, fallback на 2 маяка
        *x = (pow(d1, 2) - pow(d2, 2) + pow(BEACON_DIST_M, 2)) / (2 * BEACON_DIST_M);
        double y_squared = pow(d1, 2) - pow(*x, 2);
        *y = (y_squared > 0) ? sqrt(y_squared) : 0.0;
        return;
    }
    *x = (C * E - B * F) / denom;
    *y = (A * F - C * D) / denom;
}

// Callback-функция для обработки результатов BLE сканирования
static void ble_scan_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_SCAN_RESULT_EVT) {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            uint8_t *adv_data = scan_result->scan_rst.ble_adv;
            int adv_data_len = scan_result->scan_rst.adv_data_len;
            int rssi = scan_result->scan_rst.rssi;

            for (int i = 0; i < adv_data_len; ) {
                uint8_t len = adv_data[i];
                if (len == 0) break;
                uint8_t type = adv_data[i + 1];
                
                if (type == 0xFF && len >= 25) { // Manufacturer specific data
                    if (adv_data[i+2] == 0x4C && adv_data[i+3] == 0x00 && adv_data[i+4] == 0x02 && adv_data[i+5] == 0x15) {
                        char uuid_str[33];
                        for (int j = 0; j < 16; j++) {
                            sprintf(&uuid_str[j*2], "%02X", adv_data[i + 6 + j]);
                        }
                        uuid_str[32] = '\0';
                        
                        if (strcmp(uuid_str, COMMON_UUID) == 0) {
                            uint16_t major = (adv_data[i + 22] << 8) | adv_data[i + 23];
                            
                            if (major == BEACON1_MAJOR) {
                                beacons.rssi1 = average_rssi(beacons.rssi_history1, beacons.rssi_count1++, rssi);
                                beacons.last_seen1 = xTaskGetTickCount();
                            } else if (major == BEACON2_MAJOR) {
                                beacons.rssi2 = average_rssi(beacons.rssi_history2, beacons.rssi_count2++, rssi);
                                beacons.last_seen2 = xTaskGetTickCount();
                            } else if (major == BEACON3_MAJOR) {
                                beacons.rssi3 = average_rssi(beacons.rssi_history3, beacons.rssi_count3++, rssi);
                                beacons.last_seen3 = xTaskGetTickCount();
                            }
                        }
                    }
                }
                i += len + 1;
            }
        }
    }
}

// Инициализация BLE сканера
static void ble_init_scanner(void) {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Использование современной функции для инициализации Bluedroid
    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bd_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_scan_cb));

    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x80,  // Уменьшение чувствительности: реже сканируем (было 0x50)
        .scan_window = 0x50,    // (было 0x30)
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0));
    ESP_LOGI(TAG, "BLE Scanner initialized and started.");
}

// Обработчик событий Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying to connect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Инициализация Wi-Fi в режиме станции (STA)
static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. Waiting for connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

// Функция отправки данных о позиции на сервер
static void send_position_data(double x, double y, const char *status) {
    char post_data[128];
    snprintf(post_data, sizeof(post_data), "{\"x\":%.2f,\"y\":%.2f,\"status\":\"%s\"}", x, y, status);

    // Указываем полный URL, включая путь /post_position
    esp_http_client_config_t config = {
        .url = SERVER_URL "/post_position", 
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

// Главная задача: мониторинг маяков, расчет и отправка данных
static void monitor_task(void *arg) {
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);

    while (1) {
        TickType_t now = xTaskGetTickCount();
        bool lost1 = ((now - beacons.last_seen1) > pdMS_TO_TICKS(5000)) || (beacons.rssi1 < RSSI_THRESHOLD);
        bool lost2 = ((now - beacons.last_seen2) > pdMS_TO_TICKS(5000)) || (beacons.rssi2 < RSSI_THRESHOLD);
        bool lost3 = ((now - beacons.last_seen3) > pdMS_TO_TICKS(5000)) || (beacons.rssi3 < RSSI_THRESHOLD);
        
        const char *status = "ok";
        double x = 0.0, y = 0.0;

        if (lost1 || lost2 || lost3) {
            status = "lost";
            ESP_LOGW(TAG, "Signal lost!");
            
            gpio_set_level(BUZZER_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(BUZZER_PIN, 0);
        } else {
            // TxPower для маяков взят из их прошивок
            double d1 = calculate_distance(beacons.rssi1, -42);
            double d2 = calculate_distance(beacons.rssi2, -70);
            double d3 = calculate_distance(beacons.rssi3, -59);  // Для Beacon3 (сервер)
            trilaterate_position(&x, &y, d1, d2, d3);
            ESP_LOGI(TAG, "Pos(%.2f, %.2f) from d1=%.2fm, d2=%.2fm, d3=%.2fm", x, y, d1, d2, d3);
        }

        send_position_data(x, y, status);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Главная функция приложения
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Starting BLE Scanner...");
    ble_init_scanner();

    ESP_LOGI(TAG, "Starting WiFi Station...");
    wifi_init_sta();

    ESP_LOGI(TAG, "Creating monitor task...");
    xTaskCreate(monitor_task, "monitor_task", 8192, NULL, 5, NULL);
}