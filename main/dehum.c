/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * Based on:
 * Zigbee HA_color_dimmable_light Example
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
#include "driver/uart.h"
#include "driver/gpio.h"

#define ZB_INIT_DELAY_MS 6000
#define UART_BUF_SIZE 256

// UART talking to "display board" with display and buttons
#define UART_DISPLAY UART_NUM_1
#define PIN_TX_DISP 19
#define PIN_RX_DISP 20

// UART talking to mainboard with temp/humid sensor, and controlling fan and compressor
#define UART_MAINBOARD UART_NUM_0
#define PIN_TX_MOBO 14
#define PIN_RX_MOBO 18

#define DISP_MSG_LEN 10
#define MOBO_MSG_LEN 15

// 0x80: Power on
#define BYTE_STATE_POWER                  2
// (byt & 0x03) must be 0x02, otherwise mainboard ignores target humidity
// 0x20: Fanspeed high
#define BYTE_STATE_FAN_MAGIC              3
// Currently measured rel humidity
#define BYTE_M_HUMIDITY                   4
// Target humidity from 0x00 to 0x0e (see struct)
#define BYTE_TARGET_HUMIDITY              5
// Upper two bits = on or off timer, lower bits:
// D -> M: Hours to initialize timer to
// M -> D: Hours remaining + 1
#define BYTE_STATE_TIMER                  6
// 0x02: Enable wifi pairing
#define BYTE_D_STATE_PAIRING              7
// 0x01: Power on
// 0x80: Compressor on
#define BYTE_M_STATE_COMPRESSOR_POWER2    7
// 0x02: Reflect wifi pairing mode, or set 0 to disable LED on display
#define BYTE_M_STATE_PAIRING              8
// 0x04: Water tank full/removed
#define BYTE_M_STATE_WATER                9
// Ambient temp in °C
#define BYTE_M_TEMPERATURE               10
// Remaining time in minutes mod 60 + 1
#define BYTE_M_TIMER_MINUTES             13

#define BYTE_D_CRC                       (DISP_MSG_LEN - 1)
#define BYTE_M_CRC                       (MOBO_MSG_LEN - 1)

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

// Log info or warning depending on given esp_err_t code
#define ESP_LOGIW(TAG, err, fmt, ...)                  \
    do {                                               \
        const esp_err_t e = err;                       \
        if (e == ESP_OK) {                             \
            ESP_LOGI(TAG, fmt, ##__VA_ARGS__);         \
        } else {                                       \
            ESP_LOGW(TAG, fmt ": %s",                  \
                    ##__VA_ARGS__,                     \
                    esp_err_to_name(e));               \
        }                                              \
    } while (0)

static void handle_state(bool force);

static void hexdump(const char *prefix, const uint8_t *buffer, int actual_len, uint8_t crc);

struct uart_task
{
    QueueHandle_t queue;
    int uart;
    uint8_t *buffer;
    int blen;

    void (*cb)(void);

    const char *tag;
};

static struct
{
    bool on;
    bool pairing;
    bool fan_hi;
    bool water_full;
    bool compressor;
    uint8_t target; // AU CO 30 35 40 45 50 55 60 65 70 75 80 85 90 [00-0e]
    uint16_t humidity; // * 100
    uint16_t temp; // * 100
    uint16_t timer_mins_sum; // Remaining minutes (0..xxx), or 0 if timer disabled
} state = {.on = false, .fan_hi = true}, old_state;

static bool zigbee_ok = false;
static bool zb_pairing = false;

// To protect the two receive buffers while we read or modify them
static SemaphoreHandle_t bufferMutex = NULL;

static const char *TAG = "MAIN";
/********************* Define functions **************************/

static int slen(const char *str)
{
    const char *ptr = str;
    while ( *ptr != '\0' ) {
        ++ptr;
    }
    return (int) (ptr - str);
}

static void sncpy(char *dest, const char *src, int len)
{
    while ( len-- > 0 ) {
        *dest = *src;
        if ( *src == '\0' )
            return;
        ++src;
        ++dest;
    }
}

static esp_err_t zb_add_manufacturer_model(
    esp_zb_ep_list_t *ep_list, uint16_t endpoint_id, const char *manuf, const char *model)
{
    esp_err_t ret = ESP_OK;
    esp_zb_cluster_list_t *cluster_list = NULL;
    esp_zb_attribute_list_t *basic_cluster = NULL;
    int len;
    char buf[33];

    // Get cluster list for endpoint
    cluster_list = esp_zb_ep_list_get_ep( ep_list, endpoint_id );
    ESP_RETURN_ON_FALSE( cluster_list, ESP_ERR_INVALID_ARG, TAG,
        "Failed to find endpoint id: %d in list: %p", endpoint_id, ep_list );
    basic_cluster = esp_zb_cluster_list_get_cluster( cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE );
    ESP_RETURN_ON_FALSE( basic_cluster, ESP_ERR_INVALID_ARG, TAG,
        "Failed to find basic cluster in endpoint: %d", endpoint_id );
    // Manufacturer
    ESP_RETURN_ON_FALSE( manuf != NULL, ESP_ERR_INVALID_ARG, TAG,
        "Invalid manufacturer name: NULL" );
    len = slen( manuf );
    ESP_RETURN_ON_FALSE( len < 32, ESP_ERR_INVALID_ARG, TAG,
        "Invalid manufacturer name: too long" );
    sncpy( buf + 1, manuf, 32 );
    buf[0] = (char) len;
    ESP_ERROR_CHECK( esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, buf) );
    // Model name
    ESP_RETURN_ON_FALSE( model != NULL, ESP_ERR_INVALID_ARG, TAG,
        "Invalid model name: NULL" );
    len = slen( model );
    ESP_RETURN_ON_FALSE( len < 32, ESP_ERR_INVALID_ARG, TAG,
        "Invalid model name: too long" );
    sncpy( buf + 1, model, 32 );
    buf[0] = (char) len;
    ESP_ERROR_CHECK( esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, buf) );
    return ret;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE( esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG,
        "Failed to start Zigbee commissioning" );
}

static void zb_configure_reporting(void)
{
    const char *TAG = "REPORTING";
    esp_err_t ret;
    ESP_LOGI( TAG, "Configuring reporting" );

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
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config humidity current" );
    // Temperature (measured)
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .u.send_info = {
            .min_interval = 30,
            .max_interval = 900,
            .def_min_interval = 30,
            .def_max_interval = 900,
            .delta.u16 = 100,
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config temperature current" );
    // On/Off
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        .u.send_info = {
            .min_interval = 0,
            .max_interval = 0xFFFF,
            .delta.u32 = 0,
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config on/off" );
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
            .delta.u32 = 0,
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config fan speed" );
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
            .delta.u32 = 0,
        },
        .manuf_code = CUSTOM_MANUFACTURER_CODE,
    };
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config humidity target" );
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
            .delta.u32 = 0,
        },
        .manuf_code = CUSTOM_MANUFACTURER_CODE,
    };
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config water full" );
    // Compressor running
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = DEHUM_MFG_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = CUSTOM_ATTR_COMPRESSOR,
        .u.send_info = {
            .min_interval = 0,
            .max_interval = 0xFFFF,
            .delta.u32 = 0,
        },
        .manuf_code = CUSTOM_MANUFACTURER_CODE,
    };
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config compressor" );
    // Timer remaining minutes
    reporting_info = (esp_zb_zcl_reporting_info_t){
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_DEHUM_ENDPOINT,
        .cluster_id = DEHUM_MFG_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = CUSTOM_ATTR_TIMER_MINUTES,
        .u.send_info = {
            .min_interval = 0,
            .max_interval = 0xFFFF,
            .delta.u32 = 0,
        },
        .manuf_code = CUSTOM_MANUFACTURER_CODE,
    };
    ret = esp_zb_zcl_update_reporting_info( &reporting_info );
    ESP_LOGIW( TAG, ret, "Reporting config timer minutes" );
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    const char *TAG = "ZB_SIGHNDL";
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch ( sig_type ) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI( TAG, "Initialize Zigbee stack" );
        esp_zb_bdb_start_top_level_commissioning( ESP_ZB_BDB_MODE_INITIALIZATION );
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if ( err_status == ESP_OK ) {
            ESP_LOGI( TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non" );
            if ( esp_zb_bdb_is_factory_new() ) {
                zb_pairing = true;
                ESP_LOGI( TAG, "Start network steering" );
                esp_zb_bdb_start_top_level_commissioning( ESP_ZB_BDB_MODE_NETWORK_STEERING );
            } else {
                ESP_LOGI( TAG, "Device rebooted" );
            }
            zigbee_ok = true;
        } else {
            zb_pairing = true;
            ESP_LOGW( TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                esp_err_to_name(err_status) );
            esp_zb_scheduler_alarm( (esp_zb_callback_t) bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_INITIALIZATION, 1000 );
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if ( err_status == ESP_OK ) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id( extended_pan_id );
            zb_pairing = false;
            old_state.pairing = state.pairing;
            ESP_LOGI( TAG,
                "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx,"
                " Channel:%d, Short Address: 0x%04hx)",
                extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address() );
        } else {
            zb_pairing = true;
            ESP_LOGI( TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status) );
            esp_zb_scheduler_alarm( (esp_zb_callback_t) bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000 );
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if ( err_status == ESP_OK ) {
            if ( *(uint8_t *) esp_zb_app_signal_get_params( p_sg_p ) ) {
                ESP_LOGI( TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(),
                    *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p) );
            } else {
                ESP_LOGW( TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id() );
            }
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        esp_zb_sleep_now();
        break;
    default:
        ESP_LOGI( TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
            esp_err_to_name(err_status) );
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
    rc = esp_pm_configure( &pm_config );
#endif
    return rc;
}

static void set_power(bool on);
static void set_fan(bool high);
static void set_target(uint8_t mode);
static void set_timer(uint8_t hours);

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    static const char *TAG = "ZB_ATTRIBHNDL";
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE( message, ESP_FAIL, TAG, "Empty message" );
    ESP_RETURN_ON_FALSE( message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG,
        "Received message: error status(%d)",
        message->info.status );
    ESP_LOGI( TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
        message->info.dst_endpoint, message->info.cluster,
        message->attribute.id, message->attribute.data.size );
    if ( message->info.dst_endpoint == HA_DEHUM_ENDPOINT ) {
        switch ( message->info.cluster ) {
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            if ( message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type ==
                 ESP_ZB_ZCL_ATTR_TYPE_BOOL ) {
                bool new_state = message->attribute.data.value
                                     ? (!!*(bool *) message->attribute.data.value)
                                     : false;
                set_power( new_state );
            } else {
                ESP_LOGW( TAG, "Unknown On/Off cluster attr/type: attribute(0x%x), type(0x%x)", message->attribute.id,
                    message->attribute.data.type );
            }
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL:
            if ( message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID &&
                 message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM ) {
                uint8_t mode = message->attribute.data.value
                                   ? *(uint8_t *) message->attribute.data.value
                                   : 0xff;

                ESP_LOGI( TAG, "Fan mode set to %d", mode );

                switch ( mode ) {
                case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW:
                    set_fan( false );
                    break;
                case ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH:
                    set_fan( true );
                    break;
                default:
                    ESP_LOGW( TAG, "Unsupported fan mode: %d", mode );
                    uint8_t old_zb_mode = state.fan_hi
                                              ? ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH
                                              : ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW;
                    esp_zb_zcl_set_attribute_val( message->info.dst_endpoint,
                        message->info.cluster,
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        message->attribute.id,
                        &old_zb_mode, false );

                    return ESP_ERR_INVALID_ARG;
                }
            } else {
                ESP_LOGW( TAG, "Unknown fan cluster attr/type: attribute(0x%x), type(0x%x)", message->attribute.id,
                    message->attribute.data.type );
            }
            break;
        case DEHUM_MFG_CLUSTER_ID:
            if ( message->attribute.id == CUSTOM_ATTR_TARGET &&
                 message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM ) {
                uint8_t mode = message->attribute.data.value
                                   ? *(uint8_t *) message->attribute.data.value
                                   : 0xff;
                if ( mode < 0xf ) {
                    ESP_LOGI( TAG, "Target mode set to %d", mode );
                    set_target( mode );
                } else {
                    ESP_LOGI( TAG, "Invalid target mode %d", mode );
                    esp_zb_zcl_set_manufacturer_attribute_val( message->info.dst_endpoint,
                        message->info.cluster,
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        CUSTOM_MANUFACTURER_CODE,
                        message->attribute.id,
                        &state.target, false );
                    return ESP_ERR_INVALID_ARG;
                }
                // apply hardware behavior
            } else if ( message->attribute.id == CUSTOM_ATTR_SET_TIMER_HOURS &&
                        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 ) {
                uint8_t hours = message->attribute.data.value
                                    ? *(uint8_t *) message->attribute.data.value
                                    : 0;
                ESP_LOGI( TAG, "Timer set to %d hours", hours );
                set_timer( hours );
            } else {
                ESP_LOGW( TAG, "Unknown custom attr/type: attribute(0x%x), type(0x%x)", message->attribute.id,
                    message->attribute.data.type );
            }
            break;
        default:
            ESP_LOGI( TAG, "Unknown message: cluster(0x%x), attribute(0x%x)  ", message->info.cluster,
                message->attribute.id );
        }
    } else {
        ESP_LOGI( TAG, "Unknown endpoint(0x%x) message data: cluster(0x%x), attribute(0x%x)  ",
            message->info.dst_endpoint, message->info.cluster, message->attribute.id );
    }
    return ret;
}


static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch ( callback_id ) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler( (esp_zb_zcl_set_attr_value_message_t *) message );
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        break;
    default:
        ESP_LOGW( TAG, "Receive Zigbee action(0x%x) callback", callback_id );
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

    ESP_LOGI( TAG, "Waiting %dms before initializing zigbee stack", ZB_INIT_DELAY_MS );
    vTaskDelay( ZB_INIT_DELAY_MS / portTICK_PERIOD_MS );
    // initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    // Enable zigbee light sleep
    esp_zb_sleep_enable( true );
    esp_zb_init( &zb_nwk_cfg );

    // Dehumidifier
    //
    // create cluster lists for this endpoint
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    // Basic cluster
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
    };
    esp_zb_cluster_list_add_basic_cluster( cluster_list,
        esp_zb_basic_cluster_create( &basic_cfg ),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE );
    // On/Off
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = state.on,
    };
    esp_zb_cluster_list_add_on_off_cluster(
        cluster_list,
        esp_zb_on_off_cluster_create( &on_off_cfg ),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    );
    // Fan Control cluster
    esp_zb_fan_control_cluster_cfg_t fan_cfg = {
        .fan_mode = state.fan_hi ? ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH : ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW,
        .fan_mode_sequence = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_HIGH,
    };

    esp_zb_attribute_list_t *al = esp_zb_fan_control_cluster_create( &fan_cfg );
    // Force it this way
    for ( esp_zb_attribute_list_t *it = al; it != NULL; it = it->next ) {
        if ( it->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID ) {
            it->attribute.access |= ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
        }
    }
    esp_zb_cluster_list_add_fan_control_cluster(
        cluster_list,
        al,
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
        esp_zb_humidity_meas_cluster_create( &humidity_cfg ),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    );
    // Temperature
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = state.temp,
        .min_value = -5000,
        .max_value = 10000,
    };
    esp_zb_cluster_list_add_temperature_meas_cluster(
        cluster_list,
        esp_zb_temperature_meas_cluster_create( &temp_cfg ),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
    );
    // Custom cluster
    esp_zb_attribute_list_t *dehum_attr_list =
            esp_zb_zcl_attr_list_create( DEHUM_MFG_CLUSTER_ID );
    // Target humidity
    esp_zb_cluster_add_manufacturer_attr(
        dehum_attr_list,
        DEHUM_MFG_CLUSTER_ID,
        CUSTOM_ATTR_TARGET,
        CUSTOM_MANUFACTURER_CODE,
        ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &state.target
    );
    // Water full
    esp_zb_cluster_add_manufacturer_attr( dehum_attr_list, DEHUM_MFG_CLUSTER_ID, CUSTOM_ATTR_WATER_FULL,
        CUSTOM_MANUFACTURER_CODE,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &state.water_full );
    // Remaining timer minutes (0 if disabled)
    esp_zb_cluster_add_manufacturer_attr( dehum_attr_list, DEHUM_MFG_CLUSTER_ID, CUSTOM_ATTR_TIMER_MINUTES,
        CUSTOM_MANUFACTURER_CODE,
        ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &state.timer_mins_sum );
    // Set timer hours
    uint8_t dummy_hours = 0;
    esp_zb_cluster_add_manufacturer_attr( dehum_attr_list, DEHUM_MFG_CLUSTER_ID, CUSTOM_ATTR_SET_TIMER_HOURS,
        CUSTOM_MANUFACTURER_CODE,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_WRITE_ONLY,
        &dummy_hours );
    // Compressor state
    esp_zb_cluster_add_manufacturer_attr( dehum_attr_list, DEHUM_MFG_CLUSTER_ID, CUSTOM_ATTR_COMPRESSOR,
        CUSTOM_MANUFACTURER_CODE,
        ESP_ZB_ZCL_ATTR_TYPE_BOOL,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &state.compressor );
    // Add custom cluster to endpoint
    esp_zb_cluster_list_add_custom_cluster( cluster_list, dehum_attr_list,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE );

    // Make cluster manufacturer-specific
    for ( esp_zb_cluster_list_t *it = cluster_list; it != NULL; it = it->next ) {
        it->cluster.manuf_code = CUSTOM_MANUFACTURER_CODE;
    }

    // Configuration for this endpoint
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_DEHUM_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = HA_DEHUM_DEVICE_ID,
        .app_device_version = 0
    };

    esp_zb_ep_list_t *endpoint_list = esp_zb_ep_list_create();

    // Add the clusters + endpoint_config as a new endpoint to list
    esp_zb_ep_list_add_ep( endpoint_list, cluster_list, endpoint_config );
    zb_add_manufacturer_model( endpoint_list, HA_DEHUM_ENDPOINT,
        MANUFACTURER_NAME, MODEL_IDENTIFIER );

    // Register entire EP list
    esp_zb_device_register( endpoint_list );

    zb_configure_reporting();

    esp_zb_core_action_handler_register( zb_action_handler );
    esp_zb_set_primary_network_channel_set( ESP_ZB_PRIMARY_CHANNEL_MASK );
    ESP_ERROR_CHECK( esp_zb_start(false) );
    esp_zb_stack_main_loop();
}

static uint8_t simpleCRC(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for ( int pos = 0; pos < len; pos++ ) {
        crc += buf[pos];
    }
    return crc;
}

static void hexdump(const char *prefix, const uint8_t *buffer, int actual_len, uint8_t crc)
{
    static char pb[100];

    for ( int i = 0; i < actual_len; ++i ) {
        pb[i * 3] = ' ';
        char t = (char) (buffer[i] >> 4);
        if ( t < 10 ) t += 0x30;
        else t += 0x41 - 10;
        pb[i * 3 + 1] = t;
        t = (char) (buffer[i] & 0xf);
        if ( t < 10 ) t += 0x30;
        else t += 0x41 - 10;
        pb[i * 3 + 2] = t;
    }
    pb[actual_len * 3] = '\0';
    ESP_LOGI( TAG, "[%14s %d]: %s (%02x)", prefix, actual_len, pb, crc );
}

/*
 * UART 1, display board
 */

static uint8_t buf_display[DISP_MSG_LEN] = {0xBB, 0x51}; // e.g. BB 51 80 32 00 0B 00 00 00 BD

static void display_update_crc(void)
{
    buf_display[BYTE_D_CRC] = simpleCRC( buf_display + 2, DISP_MSG_LEN - 3 );
}

static void display_parse_message(void)
{
    old_state = state;
    // Same
    state.on = (buf_display[BYTE_STATE_POWER] & 0x80) != 0;
    state.fan_hi = (buf_display[BYTE_STATE_FAN_MAGIC] & 0x20) != 0;
    // Ignore timer from display as it doesn't have minutes
    //state.timer_mins_sum = (buf_display[BYTE_STATE_TIMER] & 0x1f) * 60;
    state.target = buf_display[BYTE_TARGET_HUMIDITY];
    state.pairing = (buf_display[BYTE_D_STATE_PAIRING] & 0x02) != 0;
}

static void display_handle(void)
{
    display_parse_message();

    if ( state.pairing ) {
        buf_display[BYTE_D_STATE_PAIRING] &= ~0x02;
        display_update_crc();
    }
    hexdump( "Relay to MOBO", buf_display, sizeof(buf_display), simpleCRC( buf_display + 2, 7 ) );
    uart_write_bytes( UART_MAINBOARD, buf_display, sizeof(buf_display) );
    handle_state( false );
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

static uint8_t buf_mobo[MOBO_MSG_LEN]; // e.g.  BB 51 80 32 2D 0B 00 43 00 00 12 00 40 00 7F


static void mobo_update_crc(void)
{
    buf_mobo[BYTE_M_CRC] = simpleCRC( buf_mobo + 2, MOBO_MSG_LEN - 3 );
}

static void mobo_parse_message(void)
{
    old_state = state;
    state.on = (buf_mobo[BYTE_STATE_POWER] & 0x80) != 0;
    state.fan_hi = (buf_mobo[BYTE_STATE_FAN_MAGIC] & 0x20) != 0;
    //
    state.humidity = (uint16_t) buf_mobo[BYTE_M_HUMIDITY] * 100;
    state.target = buf_mobo[BYTE_TARGET_HUMIDITY];
    state.compressor = (buf_mobo[BYTE_M_STATE_COMPRESSOR_POWER2] & 0x80) != 0;
    // 0x40 == comp off???
    // -> seems 0x40 and 0x80 both clear means compressor should start but won't because it's been turned on/off too rapidly
    // 0x01 == power on again?
    // 0x03 == speed high again
    state.water_full = (buf_mobo[BYTE_M_STATE_WATER] & 0x04) != 0;
    state.temp = (uint16_t) buf_mobo[BYTE_M_TEMPERATURE] * 100;
    if ( buf_mobo[BYTE_STATE_TIMER] & 0xc0 ) {
        // Either on or off timer bit set
        state.timer_mins_sum = buf_mobo[BYTE_M_TIMER_MINUTES] + (buf_mobo[BYTE_STATE_TIMER] & 0x1f) * 60;
    } else {
        state.timer_mins_sum = 0;
    }
    // Without the wifi module at least, the wifi status bit never resets once activated
    // via the display unit. However, the display panel will stop flashing the wifi light
    // after a certain while, OR when receiving a message from the mainboard that has
    // the wifi bit cleared. It won't matter whether it receives more messages later
    // that have the wifi bit set again - receiving a cleared one once is enough to turn
    // off the flashing LED on the display board.
    //state.pairing = (buf_mobo[BYTE_M_STATE_PAIRING] & 0x02) != 0;
}

static void mobo_handle(void)
{
    mobo_parse_message();
    // Mainboard is authoritative, override anything we got from display
    buf_display[BYTE_STATE_POWER] = buf_mobo[BYTE_STATE_POWER];
    buf_display[BYTE_STATE_FAN_MAGIC] = buf_mobo[BYTE_STATE_FAN_MAGIC];
    buf_display[BYTE_TARGET_HUMIDITY] = buf_mobo[BYTE_TARGET_HUMIDITY];
    buf_display[BYTE_STATE_TIMER] = buf_mobo[BYTE_STATE_TIMER];
    if ( zb_pairing && !(buf_mobo[BYTE_M_STATE_PAIRING] & 0x02) ) {
        buf_mobo[BYTE_M_STATE_PAIRING] |= 0x02;
        mobo_update_crc();
    } else if ( !zb_pairing && (buf_mobo[BYTE_M_STATE_PAIRING] & 0x02) ) {
        buf_mobo[BYTE_M_STATE_PAIRING] &= ~0x02;
        mobo_update_crc();
    }

    uart_write_bytes( UART_DISPLAY, buf_mobo, sizeof(buf_mobo) );
    handle_state( false );
}

static struct uart_task td_mobo = {
    .uart = UART_MAINBOARD,
    .buffer = buf_mobo,
    .blen = sizeof(buf_mobo),
    .cb = mobo_handle,
    .tag = "UART_MAINBOARD",
};

/*
 * Common UART handling task
 */

static void uart_task(void *pvParameters)
{
    struct uart_task *p = pvParameters;
    const char *TAG = p->tag;
    uart_event_t event;
    int ret;

    for ( ;; ) {
        //Waiting for UART event.
        if ( xQueueReceive( p->queue, (void *) &event, (TickType_t) portMAX_DELAY ) ) {
            switch ( event.type ) {
            case UART_DATA:
                xSemaphoreTake( bufferMutex, portMAX_DELAY );
                while ( event.size >= p->blen ) {
                    ret = uart_read_bytes( p->uart, p->buffer, p->blen, portMAX_DELAY );
                    if ( ret <= 0 ) {
                        ESP_LOGW( TAG, "read error %d", ret );
                        break;
                    }
                    uint8_t val = simpleCRC( p->buffer + 2, p->blen - 3 );
                    hexdump( "RECV", p->buffer, ret, val );
                    if ( val != p->buffer[ret - 1] ) {
                        ESP_LOGW( TAG, "CRC error, flushing UART" );
                        break;
                    }
                    if ( p->buffer[0] != 0xbb || p->buffer[1] != 0x51 ) {
                        ESP_LOGW( TAG, "Header wrong" );
                        break;
                    }
                    // Message seems ok, handle
                    p->cb();
                    event.size -= ret;
                }
                xSemaphoreGive( bufferMutex );
                if ( event.size > 0 ) {
                    ESP_LOGW( TAG, "Short read, %zu of %d, resetting",
                        event.size, p->blen );
                    uart_flush_input( p->uart );
                    xQueueReset( p->queue );
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI( TAG, "hw fifo overflow" );
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input( p->uart );
                xQueueReset( p->queue );
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI( TAG, "ring buffer full" );
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input( p->uart );
                xQueueReset( p->queue );
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI( TAG, "uart rx break" );
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI( TAG, "uart parity error" );
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI( TAG, "uart frame error" );
                break;
            //Others
            default:
                ESP_LOGI( TAG, "uart event type: %d", event.type );
                break;
            }
        }
    }
}

/*
 * Message injection, zigbee glue...
 */

/**
 * Handle return code by attribute update.
 * On error, release zb lock, log error, acquire lock again.
 */
static void handle_state_err(const char *name, esp_zb_zcl_status_t ret)
{
    esp_zb_lock_release();
    if ( ret != 0 ) {
        ESP_LOGW( TAG, "Updating %s failed with status %s", name, esp_err_to_name( ret ) );
    } else {
        ESP_LOGI( TAG, "Updated %s", name );
    }
    esp_zb_lock_acquire( portMAX_DELAY );
}

/**
 * Figure out if any state changed that we want to know about...
 */
static void handle_state(bool force)
{
    if ( zigbee_ok ) {
        esp_zb_zcl_status_t ret;

        esp_zb_lock_acquire( portMAX_DELAY );

        if ( force || state.on != old_state.on ) {
            ret = esp_zb_zcl_set_attribute_val( HA_DEHUM_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &state.on,
                false );
            handle_state_err( "on/off", ret );
        }

        if ( force || state.humidity != old_state.humidity ) {
            ret = esp_zb_zcl_set_attribute_val( HA_DEHUM_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &state.humidity,
                false );
            handle_state_err( "humidity", ret );
        }

        if ( force || state.temp != old_state.temp ) {
            ret = esp_zb_zcl_set_attribute_val( HA_DEHUM_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &state.temp,
                false );
            handle_state_err( "temperature", ret );
        }

        if ( force || state.fan_hi != old_state.fan_hi ) {
            uint8_t zb_mode = state.fan_hi
                                  ? ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH
                                  : ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW;
            ret = esp_zb_zcl_set_attribute_val(
                HA_DEHUM_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                &zb_mode,
                false
            );
            handle_state_err( "fanspeed", ret );
        }

        if ( force || state.water_full != old_state.water_full ) {
            ret = esp_zb_zcl_set_manufacturer_attribute_val(
                HA_DEHUM_ENDPOINT,
                DEHUM_MFG_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                CUSTOM_MANUFACTURER_CODE,
                CUSTOM_ATTR_WATER_FULL,
                &state.water_full,
                false
            );
            handle_state_err( "water full", ret );
        }

        if ( force || state.target != old_state.target ) {
            ret = esp_zb_zcl_set_manufacturer_attribute_val(
                HA_DEHUM_ENDPOINT,
                DEHUM_MFG_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                CUSTOM_MANUFACTURER_CODE,
                CUSTOM_ATTR_TARGET,
                &state.target,
                false
            );
            handle_state_err( "target", ret );
        }

        if ( force || state.timer_mins_sum != old_state.timer_mins_sum ) {
            ret = esp_zb_zcl_set_manufacturer_attribute_val(
                HA_DEHUM_ENDPOINT,
                DEHUM_MFG_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                CUSTOM_MANUFACTURER_CODE,
                CUSTOM_ATTR_TIMER_MINUTES,
                &state.timer_mins_sum,
                false
            );
            handle_state_err( "timer minutes", ret );
        }

        if ( force || state.compressor != old_state.compressor ) {
            ret = esp_zb_zcl_set_manufacturer_attribute_val(
                HA_DEHUM_ENDPOINT,
                DEHUM_MFG_CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                CUSTOM_MANUFACTURER_CODE,
                CUSTOM_ATTR_COMPRESSOR,
                &state.compressor,
                false
            );
            handle_state_err( "compressor", ret );
        }

        esp_zb_lock_release();
    }

    if ( zigbee_ok && state.pairing && !old_state.pairing && !zb_pairing ) {
        ESP_LOGI( TAG, "Entering pairing mode" );
        zb_pairing = true;
        esp_zb_lock_acquire( portMAX_DELAY );
        esp_zb_scheduler_alarm( (esp_zb_callback_t) bdb_start_top_level_commissioning_cb,
            ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000 );
        esp_zb_lock_release();
    }
}

/**
 * Update CRCs of both buffers and forward to the other board accordingly.
 * Called after updating both buffers, for when we inject a command received
 * via zigbee.
 */
static void dispatch_to_both(void)
{
    buf_display[BYTE_D_STATE_PAIRING] &= ~0x02;
    if ( zb_pairing ) {
        buf_mobo[BYTE_M_STATE_PAIRING] |= 0x02;
    } else {
        buf_mobo[BYTE_M_STATE_PAIRING] &= ~0x02;
    }
    mobo_update_crc();
    display_update_crc();
    // Pretend we received the manipulated message from the mainboard, so it will update state.*
    mobo_parse_message();
    hexdump( "To DISP", buf_mobo, sizeof(buf_mobo), 0 );
    hexdump( "To MOBO", buf_display, sizeof(buf_display), 0 );
    uart_write_bytes( UART_DISPLAY, buf_mobo, sizeof(buf_mobo) );
    uart_write_bytes( UART_MAINBOARD, buf_display, sizeof(buf_display) );
    // Update zigbee state if applicable
    handle_state( false );
}

/**
 * Set new power mode from zigbee
 */
static void set_power(bool on)
{
    if ( on == state.on )
        return;
    xSemaphoreTake( bufferMutex, portMAX_DELAY );
    // Power flag(s)
    if ( on ) {
        buf_mobo[BYTE_STATE_POWER] |= 0x80;
        buf_mobo[BYTE_M_STATE_COMPRESSOR_POWER2] |= 0x01;
        buf_display[BYTE_STATE_POWER] |= 0x80;
    } else {
        buf_mobo[BYTE_STATE_POWER] &= ~0x80;
        buf_mobo[BYTE_M_STATE_COMPRESSOR_POWER2] &= ~0x01;
        buf_display[BYTE_STATE_POWER] &= ~0x80;
    }
    // Disable timer
    buf_mobo[BYTE_STATE_TIMER] = 0;
    buf_display[BYTE_STATE_TIMER] = 0;

    dispatch_to_both();
    xSemaphoreGive( bufferMutex );
}

static void set_fan(bool high)
{
    if ( high == state.fan_hi )
        return;
    xSemaphoreTake( bufferMutex, portMAX_DELAY );
    if ( high ) {
        buf_mobo[BYTE_STATE_FAN_MAGIC] |= 0x20;
        buf_display[BYTE_STATE_FAN_MAGIC] |= 0x20;
    } else {
        buf_mobo[BYTE_STATE_FAN_MAGIC] &= ~0x20;
        buf_display[BYTE_STATE_FAN_MAGIC] &= ~0x20;
    }
    dispatch_to_both();
    xSemaphoreGive( bufferMutex );
}

static void set_target(uint8_t mode)
{
    if ( mode == state.target )
        return;
    xSemaphoreTake( bufferMutex, portMAX_DELAY );
    // If byte 3 isn't "xxxxxx10", the command gets ignored
    buf_mobo[BYTE_STATE_FAN_MAGIC] = (buf_mobo[BYTE_STATE_FAN_MAGIC] & ~0x01) | 0x02;
    buf_mobo[BYTE_TARGET_HUMIDITY] = mode;
    buf_display[BYTE_STATE_FAN_MAGIC] = (buf_display[BYTE_STATE_FAN_MAGIC] & ~0x01) | 0x02;
    buf_display[BYTE_TARGET_HUMIDITY] = mode;
    dispatch_to_both();
    xSemaphoreGive( bufferMutex );
}

static void set_timer(uint8_t hours)
{
    if ( hours > 24 ) {
        hours = 24;
    }
    xSemaphoreTake( bufferMutex, portMAX_DELAY );
    if ( hours > 0 ) {
        // Enable timer, set hours
        buf_mobo[BYTE_STATE_TIMER] = (state.on ? 0x40 : 0x80) | (hours & 0x1f);
        buf_mobo[BYTE_M_TIMER_MINUTES] = 60;
        buf_display[BYTE_STATE_TIMER] = (state.on ? 0x40 : 0x80) | (hours & 0x1f);
    } else {
        // Disable timer
        buf_mobo[BYTE_STATE_TIMER] = 0;
        buf_display[BYTE_STATE_TIMER] = 0;
    }
    dispatch_to_both();
    xSemaphoreGive( bufferMutex );
}


/**
 * Main
 */
void app_main(void)
{
    // Disable blue and RGB LED
    gpio_reset_pin( 8 );
    gpio_set_direction( 8, GPIO_MODE_OUTPUT );
    gpio_set_level( 8, 0 );
    gpio_reset_pin( 15 );
    gpio_set_direction( 15, GPIO_MODE_OUTPUT );
    gpio_set_level( 15, 0 );

    bufferMutex = xSemaphoreCreateMutex();
    ESP_RETURN_VOID_ON_FALSE( bufferMutex, TAG, "Cannot init mutex" );
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // UART for display
    ESP_ERROR_CHECK( uart_driver_install(td_display.uart, UART_BUF_SIZE, UART_BUF_SIZE, 20,
        &td_display.queue, 0) );
    ESP_ERROR_CHECK( uart_param_config(td_display.uart, &uart_config) );
    ESP_ERROR_CHECK( uart_set_pin(td_display.uart, PIN_TX_DISP, PIN_RX_DISP, -1, -1) );
    xTaskCreate( uart_task, td_display.tag, 2048, &td_display, 3, NULL );
    // UART for mainboard
    ESP_ERROR_CHECK( uart_driver_install(td_mobo.uart, UART_BUF_SIZE, UART_BUF_SIZE, 20,
        &td_mobo.queue, 0) );
    ESP_ERROR_CHECK( uart_param_config(td_mobo.uart, &uart_config) );
    ESP_ERROR_CHECK( uart_set_pin(td_mobo.uart, PIN_TX_MOBO, PIN_RX_MOBO, -1, -1) );
    xTaskCreate( uart_task, td_mobo.tag, 2048, &td_mobo, 3, NULL );
    // Zigbee
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK( nvs_flash_init() );
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK( esp_zb_power_save_init() );
    ESP_ERROR_CHECK( esp_zb_platform_config(&config) );
    xTaskCreate( esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL );
}
