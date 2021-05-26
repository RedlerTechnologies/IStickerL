#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MARKER_OPT_IGNORE_CRC_ERROR 0x01
#define MARKER_OPT_FIND_ID 0x02
#define MARKER_OPT_GET_CONTINUE 0x04
#define MARKER_OPT_GET_LAST 0x08
#define MARKER_OPT_PRINT 0x10
#define MARKER_OPT_EVENT_ID_ONLY 0x20
#define MARKER_OPT_GET_FIRST 0x40
#define MARKER_OPT_PRINT_ID 0x80

#define MARKER_ERROR_READ_FLASH_FAILED 1
#define MARKER_ERROR_BUSY 2
#define MARKER_ERROR_NOT_FOUND 3
#define MARKER_ERROR_PREMABLE 4
#define MARKER_ERROR_LENGTH 5
#define MARKER_ERROR_CRC 6
#define MARKER_ERROR_EMPTY_SECTOR 7
#define MARKER_ERROR_SYNC 8

#define FLASH_SCAN_READ_PTR_CHANGED_OK 0x0001
#define FLASH_SCAN_WRITE_PTR_RESTARTED 0x0002
#define FLASH_SCAN_READ_SET_TO_WRITE_PTR 0x0004
#define FLASH_SCAN_READ_PTR_CHANGED_CLOSELY 0x0008
#define FLASH_SCAN_READ_PTR_RESTORED_FAILED 0x0010
#define FLASH_SCAN_READ_PTR_CHANGED_AFTER_SEARCH 0x0020

#define EVENT_RECORD_OK 0
#define EVENT_RECORD_WRONG_START 1
#define EVENT_RECORD_WRONG_TYPE 2
#define EVENT_RECORD_WRONG_LENGTH 3
#define EVENT_RECORD_WRONG_CRC 4
#define EVENT_RECORD_SPLITTED 5
#define EVENT_RECORD_REMAINDER 6
#define EVENT_RECORD_EVENT_MISSING 254
#define EVENT_RECORD_EMPTY 255

#define MAX_FLASH_BUFFER 256

typedef struct {
    unsigned int   event_id;
    unsigned int   max_event_id;
    unsigned       flash_address;
    unsigned       limit_address;
    unsigned char *buffer;

    unsigned short buffer_size;
    unsigned short offset;
    unsigned short event_count;
    unsigned short search_range;

    unsigned char options;
    unsigned char error;
    unsigned char event_size;
    unsigned char empty;

} FlashMarker;

typedef struct {
    unsigned int event_id;
    unsigned int flash_address;
} xFlashMarker;

typedef struct {
    unsigned       time;
    unsigned short sector_id;
} xTimeMarker;

typedef struct {
    xFlashMarker first_marker;
    xFlashMarker max_marker;

    FlashMarker write_marker;
    FlashMarker read_marker;

    xTimeMarker time_marker;

    float time_logged_in_flash;

    unsigned last_printed_event_id;
    unsigned min_event_id;

    unsigned previous_event_id;
    unsigned missing_event_id;

    unsigned short packet_per_session;
} ScanResult;

unsigned short InitEventFlashStructure(unsigned int search_event_id);
void PrintAllEventData(void);
void InitMarker(FlashMarker *marker, unsigned char *buffer, unsigned flash_address, unsigned short buffer_size, unsigned short offset,
                unsigned char options);
void ReadEventsToMarker(FlashMarker *marker);
