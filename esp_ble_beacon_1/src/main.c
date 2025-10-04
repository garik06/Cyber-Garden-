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

// iBeacon UUID (должен совпадать с COMMON_UUID в main_hum.c)
static uint8_t beacon_uuid[16] = {
    0xFD, 0xA5, 0x06, 0x93,
    0xA4, 0xE2, 0x4F, 0xB1,
    0xAF, 0xCF, 0xC6, 0xEB,
    0x07, 0x64, 0x78, 0x25
};

// Формирование manufacturer данных (iBeacon packet)
static uint8_t manufacturer_data[25]; 

static void init_ibeacon_data(uint16_t major, uint16_t minor, int8_t tx_power) {
    int idx = 0;
    
    // Apple Company ID: 0x004C
    manufacturer_data[idx++] = 0x4C;
    manufacturer_data[idx++] = 0x00;
    
    // iBeacon Type (0x02) и Length (0x15)
    manufacturer_data[idx++] = 0x02;
    manufacturer_data[idx++] = 0x15;
    
    // UUID (16 байт)
    memcpy(&manufacturer_data[idx], beacon_uuid, 16);
    idx += 16;
    
    // Major (Big Endian)
    manufacturer_data[idx++] = (uint8_t)(major >> 8);
    manufacturer_data[idx++] = (uint8_t)(major & 0xFF);
    
    // Minor (Big Endian)
    manufacturer_data[idx++] = (uint8_t)(minor >> 8);
    manufacturer_data[idx++] = (uint8_t)(minor & 0xFF);
    
    // Measured Power (Tx Power)
    manufacturer_data[idx++] = tx_power;
}

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20, // 20 мс
    .adv_int_max = 0x40, // 40 мс
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // После установки данных, начинаем рекламу
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            ESP_LOGI(TAG, "Advertising started successfully");
        }
        break;
    default:
        break;
    }
}

void app_main(void)
{
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Инициализация Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Регистрация GAP callback
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    // Настроим iBeacon данные
    // Major=0x0001 (Beacon 1), Minor=0x0001, TxPower=-42 dBm
    ESP_LOGI(TAG, "Setting up iBeacon data: Major=0x0001, Minor=0x0001, TxPower=-42");
    init_ibeacon_data(0x0001, 0x0001, -42);

    // Упаковываем рекламу
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
        // Используем 0x06 для совместимости с вашей версией SDK (GEN_DISC | NO_BREDR)
        .flag = 0x06, 
    };

    // Задаем данные для рекламы
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
}