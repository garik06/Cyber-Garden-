#include <stdio.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "Beacon1";

// iBeacon UUID
static uint8_t beacon_uuid[16] = {
    0xFD, 0xA5, 0x06, 0x93,
    0xA4, 0xE2, 0x4F, 0xB1,
    0xAF, 0xCF, 0xC6, 0xEB,
    0x07, 0x64, 0x78, 0x25
};

// Формирование manufacturer данных вручную (iBeacon packet)
static uint8_t manufacturer_data[25]; // Уменьшено до точного размера

static void init_ibeacon_data(uint16_t major, uint16_t minor, int8_t tx_power) {
    int idx = 0;
    manufacturer_data[idx++] = 0x4C;  // Apple Company ID (Little Endian)
    manufacturer_data[idx++] = 0x00;
    manufacturer_data[idx++] = 0x02;  // iBeacon type
    manufacturer_data[idx++] = 0x15;  // iBeacon length
    memcpy(&manufacturer_data[idx], beacon_uuid, 16);
    idx += 16;
    manufacturer_data[idx++] = (major >> 8) & 0xFF;  // Major (Big Endian)
    manufacturer_data[idx++] = major & 0xFF;
    manufacturer_data[idx++] = (minor >> 8) & 0xFF;  // Minor (Big Endian)
    manufacturer_data[idx++] = minor & 0xFF;
    manufacturer_data[idx++] = tx_power;  // Tx Power

    // Отладочный лог для проверки данных
    char data_str[75];
    for (int i = 0; i < idx; i++) {
        sprintf(&data_str[i * 2], "%02X", manufacturer_data[i]);
    }
    data_str[idx * 2] = '\0';
    ESP_LOGI(TAG, "Manufacturer data: %s", data_str);
}

// Callback GAP
static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete, starting advertising");
            esp_ble_gap_start_advertising(&(esp_ble_adv_params_t) {
                .adv_int_min       = 0x20,
                .adv_int_max       = 0x40,
                .adv_type          = ADV_TYPE_NONCONN_IND,
                .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
                .channel_map       = ADV_CHNL_ALL,
                .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            });
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Advertising start failed, status: %d", param->adv_start_cmpl.status);
            }
            break;
        default:
            break;
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Beacon1");

    // Инициализация NVS
    ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS error, erasing and retrying");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Инициализация Bluetooth
    ESP_LOGI(TAG, "Initializing Bluetooth");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_LOGI(TAG, "Bluetooth controller initialized");
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_LOGI(TAG, "Bluetooth enabled in BLE mode");
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_LOGI(TAG, "Bluedroid initialized");
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_LOGI(TAG, "Bluedroid enabled");

    // Настроим iBeacon данные
    ESP_LOGI(TAG, "Setting up iBeacon data: Major=0x0001, Minor=0x0001, TxPower=-42");
    init_ibeacon_data(0x0001, 0x0001, -42);  // TX_POWER скорректирован по RSSI

    // Упаковываем рекламу
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = false,
        .include_txpower = false,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0,
        .manufacturer_len = 25,  // Точный размер данных
        .p_manufacturer_data = manufacturer_data,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = 0,  // Убраны флаги, чтобы не превышать лимит
    };

    // Запускаем GAP callback
    ESP_LOGI(TAG, "Registering GAP callback");
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));

    // Конфигурируем данные рекламы
    ESP_LOGI(TAG, "Configuring advertising data");
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

    // Устанавливаем имя маяка (для отладки)
    ESP_LOGI(TAG, "Setting device name to Beacon1");
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name("Beacon1"));

    ESP_LOGI(TAG, "Beacon1 setup complete, entering loop");
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}