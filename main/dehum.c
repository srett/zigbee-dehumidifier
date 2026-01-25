/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * Based on:
 * Zigbee HA_color_dimmable_light Example
 * and:
 * https://github.com/SFeli/ESP32_S8/blob/master/ESP32_S8_01.c
 */

#include "dehum.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#include "driver/uart.h"

#define ZB_INIT_DELAY_MS 10000
#define UART_BUF_SIZE 256
#define PIN_TX_DISP 19
#define PIN_RX_DISP 20
#define PIN_TX_MOBO 15
#define PIN_RX_MOBO 18
// UART talking to "display board" with display and buttons
#define UART_DISPLAY UART_NUM_1
// UART talking to mainboard with temp/humid sensor, and controlling fan and compressor
#define UART_MAINBOARD UART_NUM_0

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

struct uart_task {
    QueueHandle_t queue;
    int uart;
    uint8_t *buffer;
    int blen;
    void (*cb)(void);
    const char* tag;
};

static struct {
    bool on;
    bool pairing;
    bool fan_hi;
    bool water_full;
    bool compressor;
    uint16_t humidity;
    uint8_t temp;
    uint8_t target; // AU CO 30 35 40 45 50 55 60 65 70 75 80 85 90 [00-0e]
    uint8_t timer; // timer hours remaining + 1
    uint8_t timer_min; // Only valid if timer > 0
} state = { .on = false, .fan_hi = true }, old_state;

static uint8_t pb[100];

static bool zigbee_ok = false;

static const char *TAG = "MAIN";
/********************* Define functions **************************/

int slen(const char *str)
{
    const char* ptr = str;
    while (*ptr != '\0') {
        ++ptr;
    }
    return ptr - str;
}

void sncpy(char *dest, const char *src, int len)
{
    while (len-- > 0) {
        *dest = *src;
        if (*src == '\0')
            return;
        ++src;
        ++dest;
    }
}

static esp_err_t zb_add_manufacturer_model(
        esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, const char *manuf, const char *model)
{
    esp_err_t ret = ESP_OK;
    esp_zb_cluster_list_t *cluster_list = NULL;
    esp_zb_attribute_list_t *basic_cluster = NULL;
    int len;
    char buf[33];

    // Get cluster list for endpoint
    cluster_list = esp_zb_ep_list_get_ep(ep_list, endpoint_id);
    ESP_RETURN_ON_FALSE(cluster_list, ESP_ERR_INVALID_ARG, TAG,
            "Failed to find endpoint id: %d in list: %p", endpoint_id, ep_list);
    basic_cluster = esp_zb_cluster_list_get_cluster(cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_RETURN_ON_FALSE(basic_cluster, ESP_ERR_INVALID_ARG, TAG,
            "Failed to find basic cluster in endpoint: %d", endpoint_id);
    // Manufacturer
    ESP_RETURN_ON_FALSE(manuf != NULL, ESP_ERR_INVALID_ARG, TAG,
            "Invalid manufacturer name: NULL");
    len = slen(manuf);
    ESP_RETURN_ON_FALSE(len < 32, ESP_ERR_INVALID_ARG, TAG,
            "Invalid manufacturer name: too long");
    sncpy(buf + 1, manuf, 32);
    buf[0] = len;
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
                ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, buf));
    // Model name
    ESP_RETURN_ON_FALSE(model != NULL, ESP_ERR_INVALID_ARG, TAG,
            "Invalid model name: NULL");
    len = slen(model);
    ESP_RETURN_ON_FALSE(len < 32, ESP_ERR_INVALID_ARG, TAG,
            "Invalid model name: too long");
    sncpy(buf + 1, model, 32);
    buf[0] = len;
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
                ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, buf));
    return ret;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

static void zb_configure_reporting(void)
{
    const char *TAG = "REPORTING";
    esp_err_t ret;
    ESP_LOGI(TAG, "Configuring reporting");

    // Humidity (measured)
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        .u.send_info = {
            .min_interval = 30,
            .max_interval = 900,
            .def_min_interval = 30,
            .def_max_interval = 900,
            .delta.u16 = 100,
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_LOGI(TAG, "Reporting config humidity current: %s", esp_err_to_name(ret));
    // Humidity (target)
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = DEHUM_MFG_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = CUSTOM_ATTR_TARGET,
        .u.send_info = {
            .min_interval = 0,
            .max_interval = 0xFFFF,
            .delta.u8 = 1,  // report on change
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_LOGI(TAG, "Reporting config humidity target: %s", esp_err_to_name(ret));
    // Water full
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = DEHUM_MFG_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = CUSTOM_ATTR_WATER_FULL,
        .u.send_info = {
            .min_interval = 0,
            .max_interval = 0xFFFF,
            .delta.u8 = 1,  // report on change
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_LOGI(TAG, "Reporting config water full: %s", esp_err_to_name(ret));
    // Fan speed
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
        .u.send_info = {
            .min_interval = 0,
            .max_interval = 0xFFFF,
            .delta.u8 = 1,  // report on change
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_LOGI(TAG, "Reporting config fan speed: %s", esp_err_to_name(ret));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    const char *TAG = "ZB_SIGHNDL";
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
            zigbee_ok = true;
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        //ESP_LOGI(TAG, "Zigbee can sleep");
        esp_zb_sleep_now();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

void set_power(bool on)
{
    // TODO
}

void set_fan(bool high)
{
    // TODO
}

void set_target(uint8_t mode)
{
    // TODO
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    static const char *TAG = "ZB_ATTRIBHNDL";
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_DEHUM_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                bool new_state = message->attribute.data.value
                    ? (!!*(bool*)message->attribute.data.value) : false;
                set_power(new_state);
            } else {
                ESP_LOGW(TAG, "Unknown On/Off cluster attr/type: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID &&
                message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {

                uint8_t mode = message->attribute.data.value
                    ? *(uint8_t *)message->attribute.data.value : 0xff;

                ESP_LOGI(TAG, "Fan mode set to %d", mode);

                switch (mode) {
                case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW:
                    set_fan(false);
                    break;
                case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH:
                    set_fan(true);
                    break;
                default:
                    ESP_LOGW(TAG, "Unsupported fan mode: %d", mode);
                }
            } else {
                ESP_LOGW(TAG, "Unknown fan cluster attr/type: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        case DEHUM_MFG_CLUSTER_ID:
            if (message->attribute.id == CUSTOM_ATTR_TARGET &&
                message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM) {
                uint8_t mode = message->attribute.data.value
                    ? *(uint8_t *)message->attribute.data.value : 0xff;
                if (mode < 0xf) {
                    ESP_LOGI(TAG, "Target mode set to %d", mode);
                    set_target(mode);
                } else {
                    ESP_LOGI(TAG, "Invalid target mode %d", mode);
                }
                // apply hardware behavior
            } else {
                ESP_LOGW(TAG, "Unknown custom attr/type: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        default:
            ESP_LOGI(TAG, "Unknown message: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    } else {
        ESP_LOGI(TAG, "Unknown endpoint(0x%x) message data: cluster(0x%x), attribute(0x%x)  ", message->info.dst_endpoint, message->info.cluster, message->attribute.id);
    }
    return ret;
}


static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}


/**
 * Zigbee task - setup and mainloop
 */
static void esp_zb_task(void *pvParameters)
{
    const char *TAG = "ZB_TASK";

    ESP_LOGI(TAG, "Waiting %dms before initializing zigbee stack", ZB_INIT_DELAY_MS);
    vTaskDelay(ZB_INIT_DELAY_MS / portTICK_PERIOD_MS);
    // initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    // Enable zigbee light sleep
    esp_zb_sleep_enable(true);
    esp_zb_init(&zb_nwk_cfg);

    // Dehumidifier
    // 
    // create cluster lists for this endpoint
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    // Basic cluster
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
    };
    esp_zb_cluster_list_add_basic_cluster(cluster_list,
            esp_zb_basic_cluster_create(&basic_cfg),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // On/off
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,
    };
    esp_zb_cluster_list_add_on_off_cluster(
        cluster_list,
        esp_zb_on_off_cluster_create(&on_off_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    );
    // Fan Control cluster
    esp_zb_fan_control_cluster_cfg_t fan_cfg = {
        .fan_mode = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW,
        .fan_mode_sequence = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_HIGH,
    };

    esp_zb_cluster_list_add_fan_control_cluster(
        cluster_list,
        esp_zb_fan_control_cluster_create(&fan_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    );
    // Relative humidity
    esp_zb_humidity_meas_cluster_cfg_t humidity_cfg = {
        .measured_value = state.humidity,
        .min_value = 0,
        .max_value = 10000,
    };
    esp_zb_cluster_list_add_humidity_meas_cluster(
        cluster_list,
        esp_zb_humidity_meas_cluster_create(&humidity_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    );
    // Custom cluster
    esp_zb_attribute_list_t *dehum_attr_list =
        esp_zb_zcl_attr_list_create(DEHUM_MFG_CLUSTER_ID);
    // Target humidity
    esp_zb_custom_cluster_add_custom_attr(
        dehum_attr_list,
        CUSTOM_ATTR_TARGET,
        ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &state.target
    );
    // Water full
    esp_zb_custom_cluster_add_custom_attr(dehum_attr_list, CUSTOM_ATTR_WATER_FULL,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &state.water_full);
    // Add custom cluster to endpoint
    esp_zb_cluster_list_add_custom_cluster(cluster_list, dehum_attr_list,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Configuration for this endpoint
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_DEHUM_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = HA_DEHUM_DEVICE_ID,
        .app_device_version = 0
    };

    esp_zb_ep_list_t *endpoint_list = esp_zb_ep_list_create();

    // Add the clusters + endpoint_config as a new endpoint to list
    esp_zb_ep_list_add_ep(endpoint_list, cluster_list, endpoint_config);
    zb_add_manufacturer_model(endpoint_list, HA_DEHUM_ENDPOINT,
        MANUFACTURER_NAME, MODEL_IDENTIFIER);

    // Register entire EP list
    esp_zb_device_register(endpoint_list);

    zb_configure_reporting();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

static uint8_t simpleCRC(unsigned char * buf, int len)
{
  uint8_t crc = 0;
  for (int pos = 0; pos < len; pos++) {
      crc += buf[pos];
  }
  return crc;
}


static int clamp(int val, int min, int max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

void handle_state(void);

/*
 * UART 1, display board
 */

static uint8_t buf_display[10];

void display_handle(void)
{
    static const char *TAG = "D_HANDLER";
    ESP_LOGI(TAG, "D-HANDLER");

    old_state = state;
    // Same
    state.on = !!( buf_display[2] & 0x80 );
    state.fan_hi = !!( buf_display[3] & 0x20 );
    // Ignore timer from display as it doesn't have minutes
    state.pairing = !!( buf_display[7] & 0x02 );
    uart_write_bytes(UART_MAINBOARD, buf_display, sizeof(buf_display));
    handle_state();
}

static struct uart_task td_display = {
    .uart = UART_DISPLAY,
    .buffer = buf_display,
    .blen = sizeof(buf_display),
    .cb = display_handle,
    .tag = "UART_DISPLAY",
};

/*
 * UART 2, main board
 */

static uint8_t buf_mobo[15];

void mobo_handle(void)
{
    static const char *TAG = "M_HANDLER";
    ESP_LOGI(TAG, "M-HANDLER");

    old_state = state;
    state.on = !!( buf_mobo[2] & 0x80 );
    state.fan_hi = !!( buf_mobo[3] & 0x20 );
    //
    state.humidity = (uint16_t)buf_mobo[4] * 100;
    state.target = buf_mobo[5];
    state.compressor = !!( buf_mobo[7] & 0x80 );
    // 0x40 == comp off???
    // 0x01 == power on again?
    // 0x03 == speed high again
    state.water_full = !!( buf_mobo[9] & 0x04 );
    state.temp = buf_mobo[10];
    if ( buf_mobo[6] & 0xb0 ) {
        state.timer = ( buf_mobo[6] & 0x1f );
        state.timer_min = buf_mobo[13];
    } else {
        state.timer = 0;
    }
    if ( buf_mobo[8] & 0x02 ) {
        // Without the wifi module at least, the wifi status bit never resets once activated
        // via the display unit. However, the display panel will stop flashing the wifi light
        // after a certain while, OR when receiving a message from the mainboard that has
        // the wifi bit cleared. It won't matter whether it receives more messages later
        // that have the wifi bit set again - receiving a cleard one once is enough to turn
        // off the flashing LED on the display board.
        buf_mobo[8] &= ~ 0x02;
        buf_mobo[14] = simpleCRC(buf_mobo + 2, 12);
    }
    uart_write_bytes(UART_DISPLAY, buf_mobo, sizeof(buf_mobo));
    handle_state();
}

static struct uart_task td_mobo = {
    .uart = UART_MAINBOARD,
    .buffer = buf_mobo,
    .blen = sizeof(buf_mobo),
    .cb = mobo_handle,
    .tag = "UART_MAINBOARD",
};

/*
 * Common UART handling
 */

static void uart_task(void *pvParameters)
{
    struct uart_task *p = pvParameters;
    const char *TAG = p->tag;
    uart_event_t event;
    int ret;

    for ( ;; ) {
        //Waiting for UART event.
        if(xQueueReceive(p->queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
                case UART_DATA:
                    while (event.size >= p->blen) {
                        ret = uart_read_bytes(p->uart, p->buffer, p->blen, portMAX_DELAY);
                        if (ret <= 0) {
                            ESP_LOGW(TAG, "read error %d", ret);
                            break;
                        } else {
                            for (int i = 0; i < ret; ++i) {
                                pb[i*3] = ' ';
                                int t = (p->buffer[i] >> 4);
                                if (t < 10) t += 0x30;
                                else t += 0x41 - 10;
                                pb[i*3+1] = t;
                                t = (p->buffer[i] & 0xf);
                                if (t < 10) t += 0x30;
                                else t += 0x41 - 10;
                                pb[i*3+2] = t;
                            }
                            pb[ret*3] = '\0';
                            uint8_t val = simpleCRC(p->buffer + 2, p->blen - 3);
                            ESP_LOGI(TAG, "[DATA EVT %d]: %s (%02x)", ret, pb, val);
                            if (val != p->buffer[ret - 1]) {
                                ESP_LOGW(TAG, "CRC error, flushing UART");
                                break;
                            }
                            if (p->buffer[0] != 0xbb || p->buffer[1] != 0x51) {
                                ESP_LOGW(TAG, "Header wrong");
                                break;
                            }
                            p->cb();
                            event.size -= ret;
                        }
                    }
                    //uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    if (event.size > 0) {
                        ESP_LOGW(TAG, "Short read, %d of %d, resetting",
                                event.size, p->blen);
                        uart_flush_input(p->uart);
                        xQueueReset(p->queue);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(p->uart);
                    xQueueReset(p->queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(p->uart);
                    xQueueReset(p->queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}

/**
 * Handle return code by attribute update.
 * On error, release zb lock, log error, acquire lock again.
 */
static void handle_state_err(const char *name, esp_zb_zcl_status_t ret)
{
    if (ret != 0) {
        esp_zb_lock_release();
        ESP_LOGW(TAG, "Updating %s failed with status %s", name, esp_err_to_name(ret));
        esp_zb_lock_acquire(portMAX_DELAY);
    }
}

/**
 * Figure out if any state changed that we want to know about...
 */
void handle_state(void)
{
    if (zigbee_ok) {
        const uint8_t zb_mode = state.fan_hi
            ? ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH
            : ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW;

        esp_zb_lock_acquire(portMAX_DELAY);
        //
        esp_zb_zcl_status_t ret = esp_zb_zcl_set_attribute_val(HA_DEHUM_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &state.humidity,
                false);
        handle_state_err( "humidity", ret );
        esp_zb_zcl_set_attribute_val(
            HA_DEHUM_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
            &zb_mode,
            false
        );
        handle_state_err( "fanspeed", ret );
        esp_zb_zcl_set_attribute_val(
            HA_DEHUM_ENDPOINT,
            DEHUM_MFG_CLUSTER_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            CUSTOM_ATTR_WATER_FULL,
            &state.water_full,
            false
        );
        handle_state_err( "water full", ret );
        esp_zb_zcl_set_attribute_val(
            HA_DEHUM_ENDPOINT,
            DEHUM_MFG_CLUSTER_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            CUSTOM_ATTR_TARGET,
            &state.target,
            false
        );
        handle_state_err( "target", ret );
        //
        esp_zb_lock_release();
    }
    if (state.pairing && !old_state.pairing) {
        ESP_LOGI(TAG, "Resetting zigbee stuff, pairing");
        zigbee_ok = false;
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        esp_zb_lock_release();
    }
}


/**
 * Main
 */
void app_main(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config1 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // UART for display
    ESP_ERROR_CHECK(uart_driver_install(td_display.uart, UART_BUF_SIZE, UART_BUF_SIZE, 20,
                &td_display.queue, 0));
    ESP_ERROR_CHECK(uart_param_config(td_display.uart, &uart_config1));
    ESP_ERROR_CHECK(uart_set_pin(td_display.uart, PIN_TX_DISP, PIN_RX_DISP, -1, -1));
    xTaskCreate(uart_task, td_display.tag, 2048, &td_display, 3, NULL);
    // UART for mainboard
    ESP_ERROR_CHECK(uart_driver_install(td_mobo.uart, UART_BUF_SIZE, UART_BUF_SIZE, 20,
                &td_mobo.queue, 0));
    ESP_ERROR_CHECK(uart_param_config(td_mobo.uart, &uart_config1));
    ESP_ERROR_CHECK(uart_set_pin(td_mobo.uart, PIN_TX_MOBO, PIN_RX_MOBO, -1, -1));
    xTaskCreate(uart_task, td_mobo.tag, 2048, &td_mobo, 3, NULL);
    // Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
