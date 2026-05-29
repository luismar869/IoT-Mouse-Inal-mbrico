#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "Mouse_IoT";

// GPIOs
#define GPIO_RIGHT 22
#define GPIO_LEFT 23
#define GPIO_UP 19
#define GPIO_DOWN 21
#define GPIO_CLICK_L 18
#define GPIO_CLICK_M 17
#define GPIO_CLICK_R 16

#define DEVICE_NAME "ESP32_Mouse"

// HID Report Descriptor — mouse estándar (botones + X + Y)
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,
    0x09, 0x02,
    0xA1, 0x01,
    0x09, 0x01,
    0xA1, 0x00,
    // Botones
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03,
    0x75, 0x01, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x05, 0x81, 0x03,
    // Ejes X e Y
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F,
    0x75, 0x08, 0x95, 0x02, 0x81, 0x06, 0xC0, 0xC0};

// UUIDs estándar HID
#define HID_SERVICE_UUID 0x1812
#define HID_INFO_UUID 0x2A4A
#define HID_REPORT_MAP_UUID 0x2A4B
#define HID_CONTROL_POINT_UUID 0x2A4C
#define HID_PROTOCOL_MODE_UUID 0x2A4E
#define HID_REPORT_UUID 0x2A4D
#define REPORT_REF_UUID 0x2908
#define CLIENT_CONFIG_UUID 0x2902

#define GATTS_APP_ID 0

// Handle del Input Report para enviar notificaciones
static uint16_t hid_conn_id = 0;
static bool connected = false;
static uint16_t input_report_handle = 0;
static esp_gatt_if_t hid_gatts_if = 0;

// Advertising
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t service_uuid[16] = {
    0x12, 0x18, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = ESP_BLE_APPEARANCE_HID_MOUSE,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Enviar reporte HID
void send_mouse_report(uint8_t buttons, int8_t dx, int8_t dy)
{
    if (!connected)
        return;
    uint8_t report[3] = {buttons, (uint8_t)dx, (uint8_t)dy};
    esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id,
                                input_report_handle, sizeof(report), report, false);
}

// Tabla de atributos GATT
static const esp_gatts_attr_db_t hid_gatt_db[] = {
    // HID Service Declaration
    [0] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_PRI_SERVICE}, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&(uint16_t){HID_SERVICE_UUID}}},

    // HID Information
    [1] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_READ}}},
    [2] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){HID_INFO_UUID}, ESP_GATT_PERM_READ, 4, 4, (uint8_t *)&(uint8_t[]){0x11, 0x01, 0x00, 0x02}}},

    // Report Map
    [3] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_READ}}},
    [4] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){HID_REPORT_MAP_UUID}, ESP_GATT_PERM_READ, sizeof(hid_report_descriptor), sizeof(hid_report_descriptor), (uint8_t *)hid_report_descriptor}},

    // Protocol Mode
    [5] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR}}},
    [6] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){HID_PROTOCOL_MODE_UUID}, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){0x01}}},

    // HID Control Point
    [7] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_WRITE_NR}}},
    [8] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){HID_CONTROL_POINT_UUID}, ESP_GATT_PERM_WRITE, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){0x00}}},

    // Input Report
    [9] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},
    [10] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){HID_REPORT_UUID}, ESP_GATT_PERM_READ, sizeof(uint8_t) * 3, sizeof(uint8_t) * 3, (uint8_t *)&(uint8_t[]){0x00, 0x00, 0x00}}},

    // CCCD para activar notificaciones BLE
    [11] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){CLIENT_CONFIG_UUID}, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&(uint16_t){0x0000}}},

    // RRD tipo de reporte HID
    [12] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&(uint16_t){REPORT_REF_UUID}, ESP_GATT_PERM_READ, 2, 2, (uint8_t *)&(uint8_t[]){0x00, 0x01}}},
};

#define HID_DB_SIZE (sizeof(hid_gatt_db) / sizeof(hid_gatt_db[0]))

static uint16_t hid_handle_table[HID_DB_SIZE];

// Maneja eventos GATT
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    // Crea
    case ESP_GATTS_REG_EVT:
        hid_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gatts_create_attr_tab(hid_gatt_db, gatts_if, HID_DB_SIZE, 0);
        break;

    // Guarda
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK &&
            param->add_attr_tab.num_handle == HID_DB_SIZE)
        {
            memcpy(hid_handle_table, param->add_attr_tab.handles,
                   sizeof(hid_handle_table));
            input_report_handle = hid_handle_table[10];
            esp_ble_gatts_start_service(hid_handle_table[0]);
            ESP_LOGI(TAG, "Servicio HID iniciado");
        }
        break;

    // Conexion
    case ESP_GATTS_CONNECT_EVT:
        hid_conn_id = param->connect.conn_id;
        connected = true;
        ESP_LOGI(TAG, "Cliente conectado");
        esp_ble_gap_stop_advertising();
        // Parámetros de conexión para baja latencia
        esp_ble_conn_update_params_t conn_params = {
            .min_int = 6, .max_int = 12, .latency = 0, .timeout = 400};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    // Desconexion
    case ESP_GATTS_DISCONNECT_EVT:
        connected = false;
        ESP_LOGI(TAG, "Cliente desconectado, reiniciando advertising...");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

// Advertising y seguridad BLE
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    // Inicia advertising
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Advertising iniciado");
        break;

    // Seguridad: acepta todas las solicitudes de emparejamiento
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    default:
        break;
    }
}

// Inicializar BLE
static void ble_init(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
}

// Inicializar GPIOs
void init_hw(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_RIGHT) | (1ULL << GPIO_LEFT) | (1ULL << GPIO_UP) |
                        (1ULL << GPIO_DOWN) | (1ULL << GPIO_CLICK_L) | (1ULL << GPIO_CLICK_M) |
                        (1ULL << GPIO_CLICK_R),
        .pull_down_en = 1,
        .pull_up_en = 0};
    gpio_config(&io_conf);
}

// Lectura y envío de reportes
void button_scan_task(void *pvParameters)
{
    while (1)
    {
        uint8_t buttons = 0;
        int8_t dx = 0, dy = 0;

        if (gpio_get_level(GPIO_CLICK_L))
            buttons |= (1 << 0);
        if (gpio_get_level(GPIO_CLICK_R))
            buttons |= (1 << 1);
        if (gpio_get_level(GPIO_CLICK_M))
            buttons |= (1 << 2);
        if (gpio_get_level(GPIO_UP))
            dy = -10;
        if (gpio_get_level(GPIO_DOWN))
            dy = 10;
        if (gpio_get_level(GPIO_LEFT))
            dx = -10;
        if (gpio_get_level(GPIO_RIGHT))
            dx = 10;

        if (buttons || dx || dy)
        {
            ESP_LOGI(TAG, "Reporte: btn=%d dx=%d dy=%d", buttons, dx, dy);
            send_mouse_report(buttons, dx, dy);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_hw();
    ESP_LOGI(TAG, "Hardware inicializado.");
    ble_init();

    xTaskCreate(button_scan_task, "button_scan_task", 4096, NULL, 5, NULL);
}