#include "ble_istickerl.h"

#include "nrf_log_ctrl.h"
#include "nrf_ringbuf.h"
#include "nrfx_atomic.h"

#define NRF_LOG_MODULE_NAME ble_istickerl
#define NRF_LOG_LEVEL BLE_ISTICKERL_BLE_LOG_LEVEL
#include "nrfx_log.h"
NRF_LOG_MODULE_REGISTER();

#define BLE_UUID_DATA_CHARACTERISTIC 0x0010

// Cloud-Wise IStickerL 128-bit base UUID 02F80000-5562-4076-83D7-912C187A0912
#define ISTICKERL_BASE_UUID                                                                                                                \
    {                                                                                                                                      \
        0x12, 0x09, 0x7A, 0x18, 0x2C, 0x91, 0xD7, 0x83, 0x76, 0x40, 0x62, 0x55, 0x00, 0x00, 0xF8, 0x02,                                    \
    }

static void on_write(ble_istickerl_t *p_istickerl, ble_evt_t const *p_ble_evt);

void ble_istickerl_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL)) {
        NRFX_LOG_ERROR("%s NULL for context (0x%0x) ble_evt_t (0x%0x)", __func__, p_context, p_ble_evt);
        return;
    }

    ble_gap_evt_t const *p_gap_evt   = &p_ble_evt->evt.gap_evt;
    ble_istickerl_t *    p_istickerl = (ble_istickerl_t *)p_context;
    ble_istickerl_evt_t  event       = {.p_istickerl = p_istickerl};

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        NRFX_LOG_INFO("%s BLE_GAP_EVT_CONNECTED (0x%0x)", __func__, p_gap_evt->conn_handle);
        p_istickerl->conn_handle = p_gap_evt->conn_handle;
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRFX_LOG_INFO("%s BLE_GAP_EVT_DISCONNECTED (0x%0x)", __func__, p_gap_evt->conn_handle);
        p_istickerl->conn_handle = BLE_CONN_HANDLE_INVALID;
        p_istickerl->busy        = false;
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        NRFX_LOG_DEBUG("%s BLE_GAP_EVT_PHY_UPDATE_REQUEST (0x%0x)", __func__, p_gap_evt->conn_handle);
        break;

    case BLE_GATTS_EVT_WRITE:
        NRFX_LOG_DEBUG("%s BLE_GATTS_EVT_WRITE (0x%0x)", __func__, p_gap_evt->conn_handle);

        on_write(p_istickerl, p_ble_evt);
        break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        p_istickerl->busy = false;
        event.type        = BLE_ISTICKERL_EVENT_TX_COMPLETE;
        p_istickerl->event_handler(&event, NULL);
        break;

    default:
        break;
    }
}

uint32_t ble_istickerl_init(ble_istickerl_t *p_istickerl, ble_istickerl_init_t *const p_istickerl_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         istickerl_base_uuid = ISTICKERL_BASE_UUID;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_istickerl);
    VERIFY_PARAM_NOT_NULL(p_istickerl_init);

    // Initialize the service structure.
    p_istickerl->event_handler = p_istickerl_init->event_handler;

    p_istickerl->busy        = false;
    p_istickerl->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&istickerl_base_uuid, &p_istickerl->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_istickerl->uuid_type;
    ble_uuid.uuid = BLE_UUID_ISTICKERL_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_istickerl->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the Data Characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = BLE_UUID_DATA_CHARACTERISTIC;
    add_char_params.uuid_type        = p_istickerl->uuid_type;
    add_char_params.max_len          = BLE_ISTICKERL_MAX_DATA_LEN;
    add_char_params.is_var_len       = true;
    add_char_params.p_init_value     = p_istickerl_init->p_data_init_value;
    add_char_params.init_len         = p_istickerl_init->data_init_len;
    add_char_params.char_props.read  = 1;
    add_char_params.char_props.write = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_istickerl->service_handle, &add_char_params, &p_istickerl->data_handle);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

static ret_code_t value_update(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length, uint16_t value_handle)
{
    ret_code_t        err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize the gatts_value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = length;
    gatts_value.offset  = 0;
    gatts_value.p_value = data;

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s", __func__);
    NRFX_LOG_HEXDUMP_DEBUG(data, length);
#endif

    if (p_istickerl->conn_handle == BLE_CONN_HANDLE_INVALID) {
        err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, value_handle, &gatts_value);
    } else {
        err_code = sd_ble_gatts_value_set(p_istickerl->conn_handle, value_handle, &gatts_value);
    }
    if (err_code != NRF_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    return err_code;
}

static ret_code_t value_notify(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length, uint16_t value_handle)
{
    ret_code_t             err_code = NRF_SUCCESS;
    ble_gatts_hvx_params_t hvx_params;
    uint16_t               hvx_len = length;

    // Initialize the hvx_params struct.
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = data;

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
    NRFX_LOG_INFO("%s index: %u length %u", __func__, index, length);

    NRFX_LOG_HEXDUMP_INFO(data, length);
    NRF_LOG_FLUSH();
#endif

    err_code = sd_ble_gatts_hvx(p_istickerl->conn_handle, &hvx_params);
    if (err_code == NRF_ERROR_RESOURCES || err_code == NRF_ERROR_BUSY) {
        p_istickerl->busy = true;
        return err_code;
    } else if (err_code != NRF_SUCCESS) {
        NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    } else if (err_code == NRF_SUCCESS) {
        if (hvx_len != length) {
            err_code = NRF_ERROR_DATA_SIZE;
            NRFX_LOG_ERROR("%s %s", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        }
    }

    return err_code;
}

static void on_generic_sessions_cccd_write(ble_istickerl_t *p_istickerl, ble_gatts_evt_write_t const *p_evt_write,
                                           const ble_istickerl_evt_type_t enabled, const ble_istickerl_evt_type_t disabled,
                                           volatile bool *p_notify)
{
    ASSERT(p_notify);

    // CCCD written, update notification state
    ble_istickerl_evt_t event = {.p_istickerl = p_istickerl};
    NRFX_LOG_INFO("%s %x, connection handle 0x%x.", __func__, p_evt_write->data[0], p_istickerl->conn_handle);

    if (ble_srv_is_notification_enabled(p_evt_write->data)) {
        *p_notify  = true;
        event.type = enabled;
    } else {
        *p_notify  = false;
        event.type = disabled;
    }

    p_istickerl->event_handler(&event, NULL);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_istickerl          AM Service structure.
 * @param[in]   p_ble_evt     Event received from the BLE stack.
 */
static void on_write(ble_istickerl_t *p_istickerl, ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_istickerl_evt_t          event       = {.p_istickerl = p_istickerl};
    p_istickerl->conn_handle                 = p_ble_evt->evt.gap_evt.conn_handle;

    // INFO Handle write for data_handle
    if ((p_evt_write->handle == p_istickerl->data_handle.value_handle) && (p_evt_write->len > 0)) {

#ifdef BLE_ISTICKERL_DATA_HEXDUMP
        NRFX_LOG_INFO("%s", __func__);
        NRFX_LOG_HEXDUMP_DEBUG(p_evt_write->data, p_evt_write->len);
        NRF_LOG_FLUSH();
#endif

        event.type            = BLE_ISTICKERL_EVENT_DATA_WRITE;
        event.params.p_data   = (uint8_t *)p_evt_write->data;
        event.params.data_len = p_evt_write->len;
        p_istickerl->event_handler(&event, NULL);
    } else {
        NRFX_LOG_WARNING("%s unkown CCCD (0x%x) Length: %u", __func__, p_evt_write->handle, p_evt_write->len);
    }
}

ret_code_t ble_istickerl_update_data(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length)
{
    return value_update(p_istickerl, data, length, p_istickerl->data_handle.value_handle);
}
