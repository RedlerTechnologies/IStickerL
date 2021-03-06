#pragma once

#include "gfilters_algorithm.h"
#include "hal/hal_data_types.h"

typedef struct {
    uint32_t time;
    uint16_t data_len;
    uint16_t event_type;
    uint8_t *data;

    bool immediate_event;
    bool save_in_flash;
} IStickerEvent;

typedef enum {
    EVENT_TYPE_TIME_SET         = 3,
    EVENT_TYPE_START_ROUTE      = 11,
    EVENT_TYPE_END_ROUTE        = 12,
    EVENT_TYPE_MEASURE          = 14,
    EVENT_TYPE_ACCIDENT         = 16,
    EVENT_TYPE_LOG              = 22,
    EVENT_TYPE_DEBUG            = 23,
    EVENT_TYPE_VERSION          = 25,
    EVENT_TYPE_DRIVER_BEHAVIOUR = 31,
    EVENT_TYPE_BUMPER           = 32,
    EVENT_TYPE_OFFROAD_START    = 33,
    EVENT_TYPE_OFFROAD_END      = 34,
    EVENT_TYPE_DEBUG_EXT        = 36,
    EVENT_TYPE_KEEP_ALIVE       = 37,
    EVENT_TYPE_RESET_INFO       = 40,

} EventType;

/////////////////////
// Log Event codes //
/////////////////////

#define LOG_APPLICATION_START 0
#define LOG_WAKEUP_BY_ACC 1
#define LOG_WAKEUP_BY_KEEP_ALIVE 2
#define LOG_FALSE_WAKEUP 3
#define LOG_WAKEUP_BY_EXT_POWER_CONNECTED 4
#define LOG_MISSED_GPS_SAMPLE_BY_SMALL_MOVEMENT 5
#define LOG_SLEEP_BY_END_OF_TRACKING 6
#define LOG_SLEEP_BY_LOW_VOLTAGE 7
#define LOG_ENTER_GPS_ECONOMIC_MODE 8 // unused
#define LOG_EXIT_GPS_ECONOMIC_MODE 9  // unused
#define LOG_ENTER_CUT_OFF 10
#define LOG_SLEEP_BY_NO_MOVEMENT 11 // unused
#define LOG_WAKEUP_BY_POWER_CONNECTED 12
#define LOG_CLOSED_ROUTE_BY_COMMAND 13      // unused
#define LOG_ENTER_MODEM_LOW_VOLTAGE_MODE 14 // unused
#define LOG_EXIT_MODEM_LOW_VOLTAGE_MODE 15  // unused
#define LOG_EXERNAL_POWER_CONNECTED 16
#define LOG_EXERNAL_POWER_DISCONNECTED 17
#define LOG_DISCONNECT_FROM_ONLINE 18        // unused
#define LOG_DISCONNECT_FROM_ONLINE_NO_ACK 19 // unused
#define LOG_NO_MATCHED_WIFI_NETWORK_FOUND 20 // unused
#define LOG_DNS_QUERY_FAILED 21              // unused
#define LOG_TCP_CONNECTION_FAILED 22
#define LOG_UDP_CONNECTION_FAILED 23         // unused
#define LOG_TCP_WELLCOME_MESSAGE_FAILED 24   // unused
#define LOG_TCP_CONNECTION_MESSAGE_FAILED 25 // unused
//#define LOG_TCP_ROLL_BACK							26
#define LOG_START_DOWNLOADING_FIRMWARE 27 // unused
#define LOG_DOWNLOAD_FIRMWARE_SUCCESS 28
#define LOG_DOWNLOAD_FIRMWARE_ERROR_NO_CONFIRM 29 // unused
#define LOG_DOWNLOAD_FIRMWARE_ERROR_NEG_CONFIRM 30
#define LOG_DATA_SENT_BY_BLE 31
#define LOG_DATA_NOT_SENT_NOT_ENOUGH 32
#define LOG_CALIBRATION_FAILED_INSIDE_MOVEMENT 33 // unused
#define LOG_TAMPER_STATIC 34
#define LOG_REMOTE_DOWNLOAD_CANCELED 35 // unused
#define LOG_PARAMETER_UPDATED_FROM_REMOTE 36
#define LOG_ROUTE_CONTINUE_BY_GPS 37 // unused
#define LOG_FALSE_GETTING_OUTSIDE_GEOGENCE 38
#define LOG_START_ROUTE_AFTER_RESET 39
#define LOG_MODEM_CLOSE 40 // unused
#define LOG_MODEM_OPEN 41  // unused
#define LOG_BLE_CONNECTED 42
//#define LOG_BLE_INIT_FAILURE						43
#define LOG_FLASH_ERROR_DEVICE_FORMATTED 44
#define LOG_GPS_CONFIGURATION_SUCCEED 45
#define LOG_GPS_CONFIGURATION_FAILS 46
#define LOG_BLE_DISCONNECT_WITH_NO_ACTIVITY 47
//#define LOG_BLE_RESET								48	// unused
#define LOG_BLE_READ_FILE_STARTED 49
#define LOG_BLE_READ_FILE_COMPLETED 50
#define LOG_BLE_READ_FILE_ABORTED 51
#define LOG_GYRO_CONFIG_FAILED 52
#define LOG_ACC_CONFIG_FAILED 53
#define LOG_CHARGER_DISABLE 54
#define LOG_CHARGER_ENABLE 55
#define LOG_ACC_DEEP_SLEEP_CONFIG_FAILED 56
#define LOG_TEST_MODE_ON 57
#define LOG_FLASH_PTR_ERROR_SLOW_SCAN 58
#define LOG_SEND_EVENTS_BY_BLE 59
#define LOG_SEND_EVENTS_BY_GSM 60
#define LOG_BLE_SENDING_MSG_TIMEOUT 61
#define LOG_UNLOCKED_DEVICE 62
#define LOG_LOCKED_DEVICE 63
#define LOG_WAKEUP_BY_ACC_BEFORE_SLEEP 64
#define LOG_NO_GPS_FIRST_FIX 65 // unused
//#define LOG_FALSE_WAKEUP							66
#define LOG_MODEM_HOME_NETWORK 67
#define LOG_MODEM_ROAMING 68
#define LOG_MODEM_FAILED_ON_REGISTRATION 69
#define LOG_MODEM_FAILED_ON_SIM_CARD 70
#define LOG_MODEM_FAILED_ON_CREATING_PDP 71
#define LOG_MODEM_FAILED_NO_RESPONSE 72
#define LOG_MODEM_FAILED_ACCESS_DENIED 73
#define LOG_MODEM_FAILED_NO_SEARCH 74
#define LOG_MODEM_FAILED_SETTING_SECURITY_PARAMS 75
#define LOG_TAMPER_DYNAMIC 76
#define LOG_TAMPER_BY_TOO_MANY_ACCIDENTS 77
#define LOG_CONTINUE_ROUTE 78


// DEBUG CODES

#define EVENT_DEBUG_FIX_TIME 0
#define EVENT_DEBUG_GSM_TRANSMIT_REASON 1
#define EVENT_DEBUG_AVERAGE_SNR 2    // unused
#define EVENT_DEBUG_FOUND_NETWORKS 3 // unused
#define EVENT_DEBUG_CALIBRATED_ANGLE_1 4
#define EVENT_DEBUG_CALIBRATED_ANGLE_2 5
#define EVENT_DEBUG_ONLINE_TIME 6 // unused
#define EVENT_DEBUG_MODEM_REGISTER_MODE 7
#define EVENT_DEBUG_CLOSE_ROUTE_REASON 8
#define EVENT_DEBUG_BLE_DISCO_REASON 9
#define EVENT_DEBUG_ROLLBACK_REASONS 10
#define EVENT_DEBUG_UPGRADE_FILE_COMM_CRC 11
#define EVENT_DEBUG_UPGRADE_FILE_FLASH_CRC 12
#define EVENT_DEBUG_GPS_SAMPLE_TIME 13
#define EVENT_DEBUG_STAY_IN_GEOFENCE_REASON 14
#define EVENT_DEBUG_BATTERY_LEVEL_CHANGE 15
#define EVENT_DEBUG_SYNC_TIME 16
#define EVENT_DEBUG_HW_RESET_REASON 17
#define EVENT_DEBUG_FLASH_SCAN_RESULT 18
#define EVENT_DEBUG_FALSE_WAKEUP_COUNT 19
#define EVENT_DEBUG_RESET_BEFORE_SLEEP 21
/*
#define EVENT_DEBUG_USED_SECTORS			22
#define EVENT_DEBUG_ERROR_SECTORS			23
#define EVENT_DEBUG_READ_ERROR_SECTORS                  24
#define EVENT_DEBUG_SKIP_SECTORS			25
*/
#define EVENT_DEBUG_READ_ID 26
#define EVENT_DEBUG_WRITE_ID 27
#define EVENT_DEBUG_MEMORY_LOG_TIME 28
#define EVENT_DEBUG_RECEPTION_LEVEL 29
#define EVENT_DEBUG_MISSING_EVENT_ID 30
#define EVENT_DEBUG_ACC_RECORD_COMPLETE 31
#define EVENT_DEBUG_RECORD_TRANSMIT_FAILED 32
#define EVENT_DEBUG_RECORD_TRANSMIT_SUCCEED 33
#define EVENT_DEBUG_RECORD_TRANSMIT_STARTED 34

// sync time reasons
#define SYNC_TIME_BEFORE 0x0000
#define SYNC_TIME_AFTER 0x8000

#define SYNC_TIME_BY_COMMAND 1
#define SYNC_TIME_BY_GPS 2

bool CreateEvent(IStickerEvent *event);
void CreateAccidentEvent(void);
void CreateGeneralEvent(uint32_t value, uint8_t event_type, uint8_t value_size);
void CreateEndRouteEvent(uint32_t time);
void CreateVersionEvent(bool is_immediate);
void CreateDriverBehaviourEvent(GFilterConfig *event_config, GFilterState *event_state);

uint16_t CRC16_Calc(uint8_t *ptrPct, uint16_t pctLen, uint16_t crc16);
void     CreateDebugEvent(uint16_t value_type, int32_t value, bool extened);
void     CreateResetInfoEvent(void);
void     CreateMeasurementEvent(float bat_voltage, float temperature);