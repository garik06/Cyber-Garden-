#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <time.h>

#include "web_page.h" // HTML страница для root_handler

static const char *TAG = "WiFiServer";

// Последние данные
float last_x = 0, last_y = 0;
char last_status[10] = "unknown";
char last_timestamp[20] = "00:00:00";

// Обработчик событий WiFi
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "WiFi AP started successfully");
    }
}


// Инициализация WiFi AP
void wifi_init_ap(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

   ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.ap.ssid, "ESPSERVER", sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen("ESPSERVER");
    strncpy((char*)wifi_config.ap.password, "ESPSERVER", sizeof(wifi_config.ap.password));
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (strlen("ESPSERVER") == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID: %s, Password: %s", wifi_config.ap.ssid, wifi_config.ap.password);
}

// Обработчик POST
esp_err_t post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    cJSON *json = cJSON_Parse(buf);
    if (json) {
        cJSON *x = cJSON_GetObjectItem(json, "x");
        cJSON *y = cJSON_GetObjectItem(json, "y");
        cJSON *status = cJSON_GetObjectItem(json, "status");
        if (x) last_x = x->valuedouble;
        if (y) last_y = y->valuedouble;
        if (status) {
            strncpy(last_status, status->valuestring, sizeof(last_status)-1);
            last_status[sizeof(last_status)-1] = '\0';
        }

        // Время на сервере
        time_t now = time(NULL);
        struct tm *t = localtime(&now);
        strftime(last_timestamp, sizeof(last_timestamp), "%H:%M:%S", t);

        cJSON_Delete(json);
    }

    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// GET /position
esp_err_t position_handler(httpd_req_t *req)
{
    char json[256];
    snprintf(json, sizeof(json), "{\"x\":%.2f,\"y\":%.2f,\"status\":\"%s\",\"timestamp\":\"%s\"}",
             last_x, last_y, last_status, last_timestamp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// GET /
esp_err_t root_handler(httpd_req_t *req)
{
    const char* html = get_html_page();
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

void app_main(void)
{
    wifi_init_ap();

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_start(&server, &config);

    httpd_uri_t post_uri = {
        .uri = "/post_position",
        .method = HTTP_POST,
        .handler = post_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &post_uri);

    httpd_uri_t position_uri = {
        .uri = "/position",
        .method = HTTP_GET,
        .handler = position_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &position_uri);

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
