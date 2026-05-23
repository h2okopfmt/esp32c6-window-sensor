/*
 * ESP32-C6 XIAO Window/Door Contact Sensor
 * 2 Reed-Inputs on GPIO2 (D2) + GPIO18 (D10)
 * Custom Cluster 0xFC00, boolean attrs id 0+1, READ + REPORTING
 * Compatible with esp32c6_converter.js (zigbee2mqtt external converter)
 */

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "alarm_timer.h"

#include "esp_zigbee.h"
#include "ezbee/zha.h"

#include "temperature_sensor.h"

static const char *TAG = "WINDOW_SENSOR";

#define WIN_SENSOR_EP_ID    10
#define WIN_SENSOR_CLUSTER  0xFC00
#define WIN_ATTR_T1         0x0000
#define WIN_ATTR_T2         0x0001
#define GPIO_T1             GPIO_NUM_2
#define GPIO_T2             GPIO_NUM_18

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void update_and_report(uint16_t attr_id, bool value)
{
    uint8_t v = value ? 1 : 0;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_zcl_set_attr_value(WIN_SENSOR_EP_ID, WIN_SENSOR_CLUSTER, EZB_ZCL_CLUSTER_SERVER,
                           attr_id, EZB_ZCL_STD_MANUF_CODE, &v, false);
    ezb_zcl_report_attr_cmd_t cmd = {
        .cmd_ctrl = {
            .fc.direction         = EZB_ZCL_CMD_DIRECTION_TO_CLI,
            .dst_addr.addr_mode   = EZB_ADDR_MODE_SHORT,
            .dst_addr.u.short_addr= 0x0000,
            .dst_ep               = 1,
            .src_ep               = WIN_SENSOR_EP_ID,
            .cluster_id           = WIN_SENSOR_CLUSTER,
        },
        .payload = { .attr_id = attr_id },
    };
    ezb_err_t ret = ezb_zcl_report_attr_cmd_req(&cmd);
    esp_zigbee_lock_release();
    ESP_LOGI(TAG, "Report attr 0x%04x = %d (ret=0x%04x)", attr_id, v, ret);
}

static void gpio_task(void *arg)
{
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(30));  /* debounce */
            int lvl = gpio_get_level(io_num);
            bool pressed = (lvl == 0);  /* active-low: pullup, reed to GND -> closed = pressed */
            if (io_num == GPIO_T1) {
                update_and_report(WIN_ATTR_T1, pressed);
            } else if (io_num == GPIO_T2) {
                update_and_report(WIN_ATTR_T2, pressed);
            }
        }
    }
}

static esp_err_t deferred_driver_init(void)
{
    static bool inited = false;
    ESP_RETURN_ON_FALSE(!inited, ESP_OK, TAG, "Already inited");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_T1) | (1ULL << GPIO_T2),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 10, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_T1, gpio_isr_handler, (void *)GPIO_T1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_T2, gpio_isr_handler, (void *)GPIO_T2));

    /* push initial state */
    update_and_report(WIN_ATTR_T1, gpio_get_level(GPIO_T1) == 0);
    update_and_report(WIN_ATTR_T2, gpio_get_level(GPIO_T2) == 0);

    inited = true;
    return ESP_OK;
}

static void esp_zigbee_alarm_bdb_commissioning(alarm_timer_arg_t arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(arg);
    esp_zigbee_lock_release();
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Deferred driver init %s", deferred_driver_init() ? "failed" : "ok");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            if (ezb_bdb_is_factory_new()) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device reboot");
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status(0x%02x), retry", ezb_app_signal_to_string(signal_type), status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t ext_pan;
            ezb_nwk_get_extended_panid(&ext_pan);
            ESP_LOGI(TAG, "Joined network: PAN ID(0x%04hx), EXT(0x%llx), Channel(%d), Short(0x%04hx)",
                     ezb_nwk_get_panid(), ext_pan.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
        } else {
            ESP_LOGW(TAG, "Steering failed (0x%02x), retry", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
    } break;
    default:
        ESP_LOGI(TAG, "Zigbee APP Signal: %s(type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

static void custom_cluster_init(uint8_t ep_id)   { ESP_LOGI(TAG, "Custom cluster init ep=%d", ep_id); }
static void custom_cluster_deinit(uint8_t ep_id) { (void)ep_id; }

esp_err_t esp_zigbee_create_window_sensor_device(void)
{
    ezb_af_device_desc_t   dev_desc    = ezb_af_create_device_desc();
    ezb_zcl_cluster_desc_t basic_desc  = NULL;
    ezb_zcl_cluster_desc_t custom_desc = NULL;

    ezb_zcl_basic_cluster_server_config_t basic_cfg = {
        .zcl_version  = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    basic_desc = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);

    ezb_zcl_custom_cluster_config_t custom_cfg = {
        .cluster_id  = WIN_SENSOR_CLUSTER,
        .init_func   = custom_cluster_init,
        .deinit_func = custom_cluster_deinit,
    };
    custom_desc = ezb_zcl_custom_create_cluster_desc(&custom_cfg, EZB_ZCL_CLUSTER_SERVER);
    static uint8_t init_val = 0;
    ezb_zcl_custom_cluster_desc_add_attr(custom_desc, WIN_ATTR_T1, EZB_ZCL_ATTR_TYPE_BOOL,
                                         EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_REPORTING, &init_val);
    ezb_zcl_custom_cluster_desc_add_attr(custom_desc, WIN_ATTR_T2, EZB_ZCL_ATTR_TYPE_BOOL,
                                         EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_REPORTING, &init_val);

    ezb_af_ep_config_t ep_config = {
        .ep_id              = WIN_SENSOR_EP_ID,
        .app_profile_id     = EZB_AF_HA_PROFILE_ID,
        .app_device_id      = 0x0000,  /* On/Off Switch */
        .app_device_version = 0,
    };
    ezb_af_ep_desc_t ep_desc = ezb_af_create_endpoint_desc(&ep_config);
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, basic_desc));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, custom_desc));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    return ESP_OK;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
    return ESP_OK;
}

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());
    ESP_ERROR_CHECK(esp_zigbee_create_window_sensor_device());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));
    ESP_LOGI(TAG, "Start ESP Zigbee Stack (Window Sensor)");
    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
