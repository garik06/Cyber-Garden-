#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_bt.h"             // Добавлено для BLE
#include "esp_gap_ble_api.h"    // Добавлено для BLE
#include "esp_bt_main.h"        // Добавлено для BLE
#include "esp_bt_device.h"      // Добавлено для BLE

static const char *TAG = "WiFiServer";

#define WIFI_SSID "ESPSERVER"
#define WIFI_PASS "ESPSERVER"
#define MAX_STA_CONN 4

// iBeacon для сервера (как Beacon3)
static uint8_t beacon_uuid[16] = {
    0xFD, 0xA5, 0x06, 0x93,
    0xA4, 0xE2, 0x4F, 0xB1,
    0xAF, 0xCF, 0xC6, 0xEB,
    0x07, 0x64, 0x78, 0x25
};
static uint8_t manufacturer_data[25];

static void init_ibeacon_data(uint16_t major, uint16_t minor, int8_t tx_power) {
    int idx = 0;
    manufacturer_data[idx++] = 0x4C;  // Apple Company ID
    manufacturer_data[idx++] = 0x00;
    manufacturer_data[idx++] = 0x02;  // iBeacon type
    manufacturer_data[idx++] = 0x15;  // iBeacon length
    memcpy(&manufacturer_data[idx], beacon_uuid, 16);
    idx += 16;
    manufacturer_data[idx++] = (major >> 8) & 0xFF;  // Major
    manufacturer_data[idx++] = major & 0xFF;
    manufacturer_data[idx++] = (minor >> 8) & 0xFF;  // Minor
    manufacturer_data[idx++] = minor & 0xFF;
    manufacturer_data[idx++] = tx_power;  // Tx Power
}

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Beacon advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Beacon advertising start failed, status: %d", param->adv_start_cmpl.status);
            }
            break;
        default:
            break;
    }
}

float last_x = 0, last_y = 0;
char last_status[20] = "unknown";
char last_timestamp[20] = "00:00:00";
SemaphoreHandle_t position_mutex;

const char* get_html_page()
{
    const char *html = 
"<!DOCTYPE html>"
"<html>"
"<head>"
    "<title>ESP32 BLE IPS - Simple</title>"
    "<style>"
        "#map-canvas {"
            "border: 1px solid black;"
            "background-color: #f0f0f0;"
        "}"
    "</style>"
"</head>"
"<body>"
    "<h1>Indoor Positioning System (Simple Test)</h1>"
    "<div id='info-panel'>"
        "Position: <span id='position'>Waiting...</span><br>"
        "Status: <span id='status-box'>Unknown</span><br>"
        "Timestamp: <span id='timestamp'>--</span>"
    "</div>"
    "<h2>Position Map (X: 0-80sm, Y: 0-50sm)</h2>"  // Масштаб можно изменить, если зона меньше
    "<canvas id='map-canvas' width='800' height='100'></canvas>"
"<script>"
"const positionEl = document.getElementById('position');"
"const statusBoxEl = document.getElementById('status-box');"
"const timestampEl = document.getElementById('timestamp');"
"const canvas = document.getElementById('map-canvas');"
"const ctx = canvas.getContext('2d');"

"function drawMap(x, y) {"
    "ctx.clearRect(0, 0, canvas.width, canvas.height);"
    "ctx.fillStyle = '#ffffff';"
    "ctx.fillRect(0, 0, canvas.width, canvas.height);"
    "ctx.strokeStyle = '#000000';"
    "ctx.strokeRect(0, 0, canvas.width, canvas.height);"
    "ctx.font = '12px Arial';"
    "ctx.fillStyle = '#000000';"
    "ctx.fillText('0sm', 5, canvas.height - 5);"
    "ctx.fillText('80sm', canvas.width - 30, canvas.height - 5);"
    "ctx.fillText('10sm', 5, 15);"
    "ctx.fillText('0sm', 5, canvas.height - 5);"
    "if (x >= 0 && x <= 80 && y >= 0 && y <= 10) {"
        "const pixelX = (x / 80) * canvas.width;"
        "const pixelY = canvas.height - (y / 10) * canvas.height;"
        "ctx.beginPath();"
        "ctx.arc(pixelX, pixelY, 5, 0, 2 * Math.PI);"
        "ctx.fillStyle = '#ff0000';"
        "ctx.fill();"
        "ctx.stroke();"
    "}"
"}"

"function updatePosition() {"
    "fetch('/get_position')"
    ".then(response => response.json())"
    ".then(data => {"
        "positionEl.textContent = `(${data.x.toFixed(2)}sm, ${data.y.toFixed(2)}sm)`;"
        "statusBoxEl.textContent = data.status;"
        "timestampEl.textContent = data.timestamp;"
        "drawMap(data.x, data.y);"
    "})"
    ".catch(error => {"
        "console.error('Fetch error:', error);"
        "positionEl.textContent = 'Error';"
    "});"
"}"

"setInterval(updatePosition, 1000);"
"updatePosition();"
"</script>"
"</body>"
"</html>";
    return html;
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WiFi AP started successfully");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected: MAC %02x:%02x:%02x:%02x:%02x:%02x, AID=%d",
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station disconnected: MAC %02x:%02x:%02x:%02x:%02x:%02x, AID=%d",
                 event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
    }
}

void wifi_init_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = MAX_STA_CONN,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_ap finished. SSID:%s password:%s channel:%d", WIFI_SSID, WIFI_PASS, 1);
}

// Добавлено: Инициализация BLE для маяка
static void ble_init_beacon(void) {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));

    // Настроим iBeacon данные: Major=0x0003, Minor=0x0001, TxPower=-59
    init_ibeacon_data(0x0003, 0x0001, -59);

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = false,
        .include_txpower = false,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0,
        .manufacturer_len = 25,
        .p_manufacturer_data = manufacturer_data,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = 0,
    };

    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    ESP_LOGI(TAG, "Beacon3 (server) setup complete");
}

esp_err_t post_handler(httpd_req_t *req) {
    char* buf = (char*)malloc(req->content_len + 1);
    int ret = httpd_req_recv(req, buf, req->content_len);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        free(buf);
        return ESP_FAIL;
    }
    buf[req->content_len] = '\0';

    ESP_LOGI(TAG, "Received POST data: %s", buf);

    cJSON *root = cJSON_Parse(buf);
    if (root != NULL) {
        cJSON *x = cJSON_GetObjectItem(root, "x");
        cJSON *y = cJSON_GetObjectItem(root, "y");
        cJSON *status = cJSON_GetObjectItem(root, "status");

        if (x && y && status) {
            if (xSemaphoreTake(position_mutex, portMAX_DELAY) == pdTRUE) {
                last_x = (float)x->valuedouble;
                last_y = (float)y->valuedouble;
                strncpy(last_status, status->valuestring, sizeof(last_status) - 1);
                last_status[sizeof(last_status) - 1] = '\0';

                struct timeval tv;
                gettimeofday(&tv, NULL);
                struct tm *tm_info = localtime(&tv.tv_sec);
                strftime(last_timestamp, sizeof(last_timestamp), "%H:%M:%S", tm_info);

                xSemaphoreGive(position_mutex);
                ESP_LOGI(TAG, "Position updated: (%.2f, %.2f), Status: %s", last_x, last_y, last_status);
            }
        }
        cJSON_Delete(root);
    }
    free(buf);

    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_position_handler(httpd_req_t *req) {
    char json[100]; 
    
    if (xSemaphoreTake(position_mutex, portMAX_DELAY) == pdTRUE) {
        snprintf(json, sizeof(json), 
                 "{\"x\":%.2f,\"y\":%.2f,\"status\":\"%s\",\"timestamp\":\"%s\"}",
                 last_x, last_y, last_status, last_timestamp);
        
        xSemaphoreGive(position_mutex);
    } else {
        snprintf(json, sizeof(json), "{\"status\":\"error\",\"message\":\"Mutex acquisition failed\"}");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

esp_err_t root_handler(httpd_req_t *req)
{
    const char* html = get_html_page();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    position_mutex = xSemaphoreCreateMutex();
    
    wifi_init_ap();

    // Добавлено: Запуск BLE маяка
    ble_init_beacon();

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;

    ESP_LOGI(TAG, "Starting HTTP server...");
    esp_err_t ret_http = httpd_start(&server, &config);
    if (ret_http == ESP_OK) {
        ESP_LOGI(TAG, "HTTP server started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret_http));
        return;
    }

    httpd_uri_t post_uri = {
        .uri = "/post_position",
        .method = HTTP_POST,
        .handler = post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &post_uri);

    httpd_uri_t get_position_uri = {
        .uri = "/get_position",
        .method = HTTP_GET,
        .handler = get_position_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &get_position_uri);

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);

    ESP_LOGI(TAG, "Server configuration complete. Access at http://192.168.4.1");
}