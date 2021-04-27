#pragma once

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "sdk_config.h"

#include <stdbool.h>
#include <stdint.h>

#define COMMAND_CHAR_MAX_LEN 64
#define ACC_CHAR_MAX_LEN 16
#define EVENTS_CHAR_MAX_LEN 64
#define MEASURE_CHAR_MAX_LEN 16
#define STATUS_CHAR_MAX_LEN 32

#define BLE_ISTICKERL_DEF(_name)                                                                                                           \
    static ble_istickerl_t _name;                                                                                                          \
    NRF_SDH_BLE_OBSERVER(_name##_obs, BLE_ISTICKERL_BLE_OBSERVER_PRIO, ble_istickerl_on_ble_evt, &_name)

// Maximum length of data (in bytes) that can be transmitted to the peer by the AM service module
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
#define BLE_ISTICKERL_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3 /* ATT Header = 1 byte OPCODE + 2 bytes Handle */)
#else
#error NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

// AM Service event type
typedef enum {
    BLE_ISTICKERL_EVENT_NONE,

    BLE_ISTICKERL_EVENT_TX_COMPLETE,

    BLE_ISTICKERL_EVENT_COMMAND_WRITE,

    BLE_ISTICKERL_COMMAND_NOTIFICATION_STARTED,
    BLE_ISTICKERL_COMMAND_NOTIFICATION_STOPPED,

    BLE_ISTICKERL_MEASURE_NOTIFICATION_STARTED,
    BLE_ISTICKERL_MEASURE_NOTIFICATION_STOPPED,

    BLE_ISTICKERL_ACC_NOTIFICATION_STARTED,
    BLE_ISTICKERL_ACC_NOTIFICATION_STOPPED,

    BLE_ISTICKERL_STATUS_NOTIFICATION_STARTED,
    BLE_ISTICKERL_STATUS_NOTIFICATION_STOPPED,

    BLE_ISTICKERL_EVENT_NOTIFICATION_STARTED,
    BLE_ISTICKERL_EVENT_NOTIFICATION_STOPPED,

    BLE_ISTICKERL_FILE_TRANSFER_NOTIFICATION_STARTED,
    BLE_ISTICKERL_FILE_TRANSFER_NOTIFICATION_STOPPED,

} ble_istickerl_evt_type_t;

#define BLE_UUID_ISTICKERL_SERVICE 0xD124 // The UUID of the IStickerL Service

typedef struct ble_istickerl_s ble_istickerl_t;

typedef struct {
    ble_istickerl_evt_type_t type;        // Event type
    ble_istickerl_t *        p_istickerl; // A pointer to the instance
    uint16_t                 conn_handle; // Connection handle

    union {
        struct {
            uint8_t *p_command;
            uint8_t  command_len;
        };
    } params;
} ble_istickerl_evt_t;

typedef void (*ble_istickerl_event_handler_t)(ble_istickerl_evt_t *p_evt, void *p_context);

typedef struct ble_istickerl_init_s {
    ble_istickerl_event_handler_t event_handler;

    uint8_t *p_command_init_value;
    uint16_t command_init_len;

    uint8_t *p_data_init_value;
    uint16_t data_init_len;
} ble_istickerl_init_t;

struct ble_istickerl_s {
    uint8_t  uuid_type;      // UUID type for IStickerL Service Base UUID
    uint16_t service_handle; // Handle of IStickerL Service (SoftDevice)

    ble_istickerl_event_handler_t event_handler; // Event handler to be called for handling received data
    uint16_t conn_handle; // Handle of the current connection (SoftDevice), will be set to BLE_CONN_HANDLE_INVALID if not in a connection

    bool busy; // Busy flag. Indicates that the hvx function returned busy and that there is still data to be transfered

    // IStickerL service Characteristics
    ble_gatts_char_handles_t command_handle;
    ble_gatts_char_handles_t acc_handle;
    ble_gatts_char_handles_t status_handle;
    ble_gatts_char_handles_t measurement_handle;
    ble_gatts_char_handles_t event_handle;
    ble_gatts_char_handles_t file_transfer_handle;
};

uint32_t ble_istickerl_init(ble_istickerl_t *p_istickerl, ble_istickerl_init_t *const p_istickerl_init);

/**@brief   Function for handling the IStickerL Service's BLE events.
 *
 * @details The IStickerL Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the IStickerL Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     IStickerL Service structure.
 */
void ble_istickerl_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

ret_code_t ble_istickerl_notify_command(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_notify_measurement(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_notify_acc(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_notify_status(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_notify_event(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_notify_file_transfer(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);

ret_code_t ble_istickerl_update_measurement(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_update_acc(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_update_status(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_update_event(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);
ret_code_t ble_istickerl_update_file_transfer(ble_istickerl_t *p_istickerl, uint8_t *const data, size_t length);