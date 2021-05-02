#include "ble_services_manager.h"

#include "FreeRTOS.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble/ble_istickerl.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_dis.h"
#include "ble_task.h"
#include "logic/commands.h"
#include "logic/configuration.h"
#include "logic/peripherals.h"
#include "logic/serial_comm.h"
#include "logic/tracking_algorithm.h"
#include "nrf_ble_gatt.h"
#include "nrf_log_ctrl.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdm.h"
#include "semphr.h"
#include "task.h"
#include "version.h"

#include <string.h>

extern DeviceConfiguration  device_config;
extern DriverBehaviourState driver_behaviour_state;

xSemaphoreHandle ble_command__notify_semaphore;

void SetDeviceIDFromMacAddress(ble_gap_addr_t *mac_address);
void SetBleID(uint8_t *dev_id);

#define NRF_LOG_MODULE_NAME ble_services_manager
#define NRF_LOG_LEVEL CLOUD_WISE_DEFAULT_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

// Time from initiating event (connect or start of notification) to first time
// sd_ble_gap_conn_param_update is called (ms)
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)
// Time between each call to sd_ble_gap_conn_param_update after the first call (ms)
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000)
// Number of attempts before giving up the connection parameter negotiation
#define MAX_CONN_PARAMS_UPDATE_COUNT 3

#define APP_ADV_FAST_INTERVAL 100 // 400 // ????????? // The advertising interval (in units of 0.625 ms. This value corresponds to 250 ms)
#define APP_ADV_FAST_TIMEOUT_IN_SECONDS (60 * 100) // The Fast advertising duration in units of 10 milliseconds

#define APP_ADV_SLOW_INTERVAL 1800                 // The advertising interval (in units of 0.625 ms. This value corresponds to 562.5 ms)
#define APP_ADV_SLOW_TIMEOUT_IN_SECONDS (60 * 100) // The Slow advertising duration in units of 10 milliseconds

#define APP_BLE_CONN_CFG_TAG 1  // A tag identifying the SoftDevice BLE configuration
#define APP_BLE_OBSERVER_PRIO 3 // Application's BLE observer priority. You shouldn't need to modify this value

#define MIN_CONN_INTERVAL (uint16_t) MSEC_TO_UNITS(7.5, UNIT_1_25_MS) // Minimum acceptable connection interval (0.001 seconds)
#define MAX_CONN_INTERVAL (uint16_t) MSEC_TO_UNITS(200, UNIT_1_25_MS) // Maximum acceptable connection interval (0.375 seconds)
#define CONN_SUP_TIMEOUT (uint16_t) MSEC_TO_UNITS(4000, UNIT_10_MS)   // Connection supervisory timeout (2 seconds)
#define SLAVE_LATENCY 0                                               // Slave latency

BLE_ADVERTISING_DEF(m_advertising); // Advertising module instance
NRF_BLE_GATT_DEF(m_gatt);           // GATT module instance
BLE_BAS_DEF(m_battery_service);     // Battery service

BLE_ISTICKERL_DEF(m_istickerl); // Cloud-Wise IStickerL service

static void init_device_information_service(void);
static void init_battery_service(void);
static void init_istickerl_service(void);

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context);
static void istickerl_evt_handler(ble_istickerl_evt_t *p_evt, void *p_context);

static void gatt_init(void);
static void gap_params_init(void);
static void advertising_init(void);
static void conn_params_init(void);
static void conn_evt_len_ext_set(bool status);

static void dfu_init(void);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; // Handle of the current connection

// Universally unique service identifiers.
static ble_uuid_t m_rsp_uuids[] = {
    {BLE_UUID_ISTICKERL_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN},
};
/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went
 * wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) { APP_ERROR_HANDLER(nrf_error); }

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code           = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void sdh_task_hook(void *p_context)
{
    //
}

void ble_services_init_0(void)
{
    static ble_gap_addr_t mac_address;

    uint32_t res;

    ble_stack_init();

    res = sd_ble_gap_addr_get(&mac_address);

    if (res == NRF_SUCCESS) {
        SetDeviceIDFromMacAddress(&mac_address);
        SetBleID(device_config.DeviceID);
    }
}

void ble_services_init(void)
{
    gap_params_init();
    gatt_init();

    init_device_information_service();
    init_istickerl_service();

    init_battery_service();

    dfu_init();

    advertising_init();

    conn_params_init();

    nrf_sdh_freertos_init(sdh_task_hook, NULL);
}

static void init_device_information_service(void)
{
    ret_code_t     err_code;
    ble_dis_init_t dis_init = {0};

#ifdef MANUFACTURER_NAME
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
#endif
#ifdef MODEL_NUM
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);
#endif
#ifdef HARDWARE_REV
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, HARDWARE_REV);
#endif
#ifdef FIRMWARE_REV
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FIRMWARE_REV);
#endif
#ifdef SOFTWARE_REV
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, SOFTWARE_REV);
#endif
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, device_serial_number);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection
 * Parameters Module which are passed to the application.
 *          @note All this function does is to disconnect. This could have been
 * done by simply setting the disconnect_on_fail config parameter, but instead
 * we use the event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed
 * to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
        NRFX_LOG_INFO("%s Fast advertising", __func__);

        break;

    case BLE_ADV_EVT_SLOW:
        NRFX_LOG_INFO("%s Slow advertising", __func__);
        break;

    case BLE_ADV_EVT_IDLE:
        NRFX_LOG_INFO("%s Idle advertising", __func__);
        break;

    default:
        break;
    }
}

static void advertising_config_get(ble_adv_modes_config_t *p_config)
{
    *p_config = (ble_adv_modes_config_t){0};

    p_config->ble_adv_fast_enabled = true;

    // TODO Enable peer manager whitelist
    // p_config->ble_adv_whitelist_enabled = true;

    p_config->ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_FAST_TIMEOUT_IN_SECONDS;

    if (APP_ADV_SLOW_TIMEOUT_IN_SECONDS > 0) {
        p_config->ble_adv_slow_enabled  = true;
        p_config->ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
        p_config->ble_adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT_IN_SECONDS;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t               err_code;
    ble_advertising_init_t   init = {0};
    ble_advdata_manuf_data_t adv_manuf_data;

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_rsp_uuids) / sizeof(m_rsp_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_rsp_uuids;

    advertising_config_get(&init.config);

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;
    uint8_t              rssi;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_DISCONNECTED:
        NRFX_LOG_INFO("Disconnected.");
        NRFX_LOG_INFO("%s Connection 0x%x has been disconnected. Reason: 0x%X", __func__, p_ble_evt->evt.gap_evt.conn_handle,
                      p_ble_evt->evt.gap_evt.params.disconnected.reason);

        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_CONNECTED:
        NRFX_LOG_INFO("Connected.");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        driver_behaviour_state.last_ble_connected_time = xTaskGetTickCount();
        break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRFX_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRFX_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRFX_LOG_INFO("PHY update request.");
        ble_gap_phys_t const phys = {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    } break;

    default:
        // No implementation needed.
        break;
    }
}

bool ble_services_is_connected(void) { return !(m_conn_handle == BLE_CONN_HANDLE_INVALID); }

/**@brief Function for starting advertising.
 */
void ble_services_advertising_start(void)
{
    ret_code_t err_code;

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 * parameters of the device including the device name, appearance, and the
 * preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    // err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)device_config.DeviceName, strlen(device_config.DeviceName));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void conn_evt_len_ext_set(bool status)
{
    ret_code_t err_code;
    ble_opt_t  opt = {0};

    opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}

static void disconnect(uint16_t conn_handle, void *p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    } else {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

/**@brief Function for handling DFU events
 *
 * @details This function is called when entering buttonless DFU
 *
 * @param[in] event
 */
static void ble_dfu_buttonless_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event) {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        NRF_LOG_INFO("Device is preparing to enter bootloader mode\r\n");
        {
            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);

            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
        }
        break;
    case BLE_DFU_EVT_BOOTLOADER_ENTER:
        NRF_LOG_INFO("Device will enter bootloader mode\r\n");
        break;

    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
        NRF_LOG_ERROR("Device failed to enter bootloader mode\r\n");
        break;

    default:
        NRF_LOG_INFO("Unknown event from ble_dfu.\r\n");
        break;
    }
}

static void dfu_init(void)
{
    // Initialize the DFU service
    ble_dfu_buttonless_init_t dfus_init = {.evt_handler = ble_dfu_buttonless_evt_handler};

    ret_code_t err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}

static void init_istickerl_service(void)
{
    ret_code_t           err_code;
    ble_istickerl_init_t istickerl_init = {0};

    istickerl_init.event_handler = istickerl_evt_handler;

    // TODO set initial data value
    istickerl_init.p_data_init_value = NULL;
    istickerl_init.data_init_len     = 0;

    err_code = ble_istickerl_init(&m_istickerl, &istickerl_init);
    APP_ERROR_CHECK(err_code);
}

static void istickerl_evt_handler(ble_istickerl_evt_t *p_evt, void *p_context)
{
    ASSERT(p_evt);

    switch (p_evt->type) {
    case BLE_ISTICKERL_EVENT_TX_COMPLETE:
        NRFX_LOG_DEBUG("%s BLE_ISTICKERL_EVENT_TX_COMPLETE", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_EVENT_COMMAND_WRITE:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_EVENT_COMMAND_WRITE (%u bytes)", __func__, p_evt->params.command_len);

        PostBleCommand(p_evt->params.p_command, p_evt->params.command_len);
        /*
        DisplayMessage(p_evt->params.p_command, p_evt->params.command_len);
        DisplayMessage("\r\n", 2);
        command_decoder(p_evt->params.p_command, p_evt->params.command_len, 1);
        */

        break;

    case BLE_ISTICKERL_COMMAND_NOTIFICATION_STARTED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_COMMAND_NOTIFICATION_STARTED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_COMMAND_NOTIFICATION_STOPPED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_COMMAND_NOTIFICATION_STOPPED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_MEASURE_NOTIFICATION_STARTED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_MEASURE_NOTIFICATION_STARTED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_MEASURE_NOTIFICATION_STOPPED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_MEASURE_NOTIFICATION_STOPPED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_ACC_NOTIFICATION_STARTED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_ACC_NOTIFICATION_STARTED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_ACC_NOTIFICATION_STOPPED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_ACC_NOTIFICATION_STOPPED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_STATUS_NOTIFICATION_STARTED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_STATUS_NOTIFICATION_STARTED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_STATUS_NOTIFICATION_STOPPED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_STATUS_NOTIFICATION_STOPPED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_EVENT_NOTIFICATION_STARTED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_EVENT_NOTIFICATION_STARTED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_EVENT_NOTIFICATION_STOPPED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_EVENT_NOTIFICATION_STOPPED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_FILE_TRANSFER_NOTIFICATION_STARTED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_FILE_TRANSFER_NOTIFICATION_STARTED", __func__);
        // TODO
        break;

    case BLE_ISTICKERL_FILE_TRANSFER_NOTIFICATION_STOPPED:
        NRFX_LOG_INFO("%s BLE_ISTICKERL_FILE_TRANSFER_NOTIFICATION_STOPPED", __func__);
        // TODO
        break;

    default:
        NRFX_LOG_WARNING("%s other (0x%x)", __func__, p_evt->type);
        break;
    }
}

static void init_battery_service(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init = {0};

    bas_init.initial_batt_level   = peripherals_read_battery_level();
    bas_init.support_notification = true;

    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_battery_service, &bas_init);
    APP_ERROR_CHECK(err_code);
}

void ble_services_update_battery_level(uint8_t battery_level)
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        ble_bas_battery_level_update(&m_battery_service, battery_level, NULL);
    } else {
        ble_bas_battery_level_update(&m_battery_service, battery_level, m_conn_handle);
    }
}

bool ble_services_notify_command(uint8_t *const data, size_t length)
{
    ret_code_t err_code;

    xSemaphoreTake(ble_command__notify_semaphore, portMAX_DELAY);

    // if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
    //    NRFX_LOG_WARNING("%s Data drop (%u bytes)", __func__, length);
    //    return false;
    //}

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s tx_data  size: %u", __func__, length);
    if (length > 0)
        NRFX_LOG_HEXDUMP_INFO(data, length);
    NRF_LOG_FLUSH();
#endif

    err_code = ble_istickerl_notify_command(&m_istickerl, data, length);

    xSemaphoreGive(ble_command__notify_semaphore);

    return (err_code == NRF_SUCCESS);
}

bool ble_services_notify_measurement(uint8_t *const data, size_t length)
{
    // if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
    //    NRFX_LOG_WARNING("%s Data drop (%u bytes)", __func__, length);
    //    return false;
    //}

    ret_code_t err_code;

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s tx_data  size: %u", __func__, length);
    if (length > 0)
        NRFX_LOG_HEXDUMP_INFO(data, length);
    NRF_LOG_FLUSH();
#endif

    err_code = ble_istickerl_notify_measurement(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_notify_acc(uint8_t *const data, size_t length)
{
    // if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
    //    NRFX_LOG_WARNING("%s Data drop (%u bytes)", __func__, length);
    //    return false;
    //}

    ret_code_t err_code;

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s tx_data  size: %u", __func__, length);
    if (length > 0)
        NRFX_LOG_HEXDUMP_INFO(data, length);
    NRF_LOG_FLUSH();
#endif

    err_code = ble_istickerl_notify_acc(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_notify_status(uint8_t *const data, size_t length)
{
    // if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
    //    NRFX_LOG_WARNING("%s Data drop (%u bytes)", __func__, length);
    //    return false;
    //}

    ret_code_t err_code;

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s tx_data  size: %u", __func__, length);
    if (length > 0)
        NRFX_LOG_HEXDUMP_INFO(data, length);
    NRF_LOG_FLUSH();
#endif

    err_code = ble_istickerl_notify_status(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_notify_event(uint8_t *const data, size_t length)
{
    // if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
    //    NRFX_LOG_WARNING("%s Data drop (%u bytes)", __func__, length);
    //    return false;
    //}

    ret_code_t err_code;

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s tx_data  size: %u", __func__, length);
    if (length > 0)
        NRFX_LOG_HEXDUMP_INFO(data, length);
    NRF_LOG_FLUSH();
#endif

    err_code = ble_istickerl_notify_event(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_notify_file_transfer(uint8_t *const data, size_t length)
{
    // if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
    //    NRFX_LOG_WARNING("%s Data drop (%u bytes)", __func__, length);
    //    return false;
    //}

    ret_code_t err_code;

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s tx_data  size: %u", __func__, length);
    if (length > 0)
        NRFX_LOG_HEXDUMP_INFO(data, length);
    NRF_LOG_FLUSH();
#endif

    err_code = ble_istickerl_notify_file_transfer(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_update_acc(uint8_t *const data, size_t length)
{
    ret_code_t err_code;

    err_code = ble_istickerl_update_acc(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_update_status(uint8_t *const data, size_t length)
{
    ret_code_t err_code;

    err_code = ble_istickerl_update_status(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_update_measurement(uint8_t *const data, size_t length)
{
    ret_code_t err_code;

    err_code = ble_istickerl_update_measurement(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_update_event(uint8_t *const data, size_t length)
{
    ret_code_t err_code;

    err_code = ble_istickerl_update_event(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

bool ble_services_update_file_transfer(uint8_t *const data, size_t length)
{
    ret_code_t err_code;

    err_code = ble_istickerl_update_file_transfer(&m_istickerl, data, length);
    return (err_code == NRF_SUCCESS);
}

void SetDeviceIDFromMacAddress(ble_gap_addr_t *mac_address)
{
    int8_t  i = 0, j = 0;
    uint8_t ch, c1, c2;

    memset(device_config.DeviceID, 0x00, 28);

    for (i = 5; i >= 0; i--) {
        ch = mac_address->addr[i];

        c1 = ch & 0x0F;
        c2 = (ch & 0xF0) >> 4;

        if (c1 < 10)
            c1 += 48;
        else
            c1 += 55;

        if (c2 < 10)
            c2 += 48;
        else
            c2 += 55;

        device_config.DeviceID[j * 2]     = c2;
        device_config.DeviceID[j * 2 + 1] = c1;
        j++;
    }

    memcpy(device_config.DeviceID + 12, device_config.DeviceID, 12);
}

void SetBleID(uint8_t *dev_id)
{
    uint8_t *dev_name = device_config.DeviceName;
    uint8_t  i, j;
    uint16_t x;

    memset(dev_name, 0x00, 16);

    dev_name[0] = 'S';
    dev_name[1] = '-';
    dev_name[2] = 'L';

    j = 0;

    for (i = 3; i < 6; i++) {
        x           = dev_id[2 * j] + 256 * dev_id[2 * j + 1];
        x           = x % 26;
        dev_name[i] = (x + 'A');
        j++;
    }

    j = 6;
    for (i = 6; i < 12; i++) {
        x           = dev_id[j];
        x           = x % 10;
        dev_name[i] = (x + '0');
        j++;
    }
}