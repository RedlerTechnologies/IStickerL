#pragma once

#include "tracking_algorithm.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef ACC_SAMPLE_FREQ_100HZ
#define ACC_SAMPLE_FREQ 100
#define RECORD_TIME_BEFORE_EVENT 12 // in tenths of seconds
#define RECORD_TIME_AFTER_EVENT 35  // in tenths of seconds
#endif

#ifdef ACC_SAMPLE_FREQ_200HZ
#define ACC_SAMPLE_FREQ 200
#define RECORD_TIME_BEFORE_EVENT 7 // in tenths of seconds
#define RECORD_TIME_AFTER_EVENT 27 // in tenths of seconds
#endif

#ifdef ACC_SAMPLE_FREQ_400HZ
#define ACC_SAMPLE_FREQ 400
#define RECORD_TIME_BEFORE_EVENT 5 // in tenths of seconds
#define RECORD_TIME_AFTER_EVENT 12 // in tenths of seconds
#endif

#define SAMPLE_PERIOD (1000 / ACC_SAMPLE_FREQ)

#define RECORD_TIME (RECORD_TIME_BEFORE_EVENT + RECORD_TIME_AFTER_EVENT)

#define NUM_SAMPLE_BLOCK_BEFORE_EVENT (ACC_SAMPLE_FREQ * RECORD_TIME_BEFORE_EVENT / 10 / SAMPLE_BUFFER_SIZE)
#define NUM_SAMPLE_BLOCK_AFTER_EVENT (ACC_SAMPLE_FREQ * RECORD_TIME_AFTER_EVENT / 10 / SAMPLE_BUFFER_SIZE)
#define NUM_SAMPLE_BLOCK_IN_MEMORY (NUM_SAMPLE_BLOCK_BEFORE_EVENT + NUM_SAMPLE_BLOCK_AFTER_EVENT)

#define SAMPLE_MEMORY_SIZE (SAMPLE_BUFFER_SIZE * NUM_SAMPLE_BLOCK_IN_MEMORY * 2 * 3)

#define RECORD_SIZE (FLASH_SECTOR_SIZE)
#define MAX_RECORDS 64
#define RECORD_AREA_FLASH_SIZE (MAX_RECORDS * RECORD_SIZE)

#define FLASH_RECORDS_START_ADDRESS (END_OF_FLASH - RECORD_AREA_FLASH_SIZE)

#define RECORD_BUFFER_SAMPLE_SIZE 40
#define RECORD_HEADER_SIZE 24
#define RECORD_TERMINATOR_SIZE 8

#define RECORD_SAMPLE_FREQ_50HZ 50
#define RECORD_SAMPLE_FREQ_100HZ (20 | 0x40)
#define RECORD_SAMPLE_FREQ_200HZ (40 | 0x40)
#define RECORD_SAMPLE_FREQ_400HZ (16 | 0x80)

#define REACORD_PARAM_ACC_X 0x01
#define REACORD_PARAM_ACC_Y 0x02
#define REACORD_PARAM_ACC_Z 0x04

#define RECORD_RESOLUTION 12

#define REACORD_FLAGS_CALIBRATED 0x01

#define RECORD_CLOSE_IND 0
#define RECORD_SENT_IND 1

typedef struct {

    AccConvertedSample samples_before_event[NUM_SAMPLE_BLOCK_BEFORE_EVENT][SAMPLE_BUFFER_SIZE];
    AccConvertedSample samples_after_event[NUM_SAMPLE_BLOCK_AFTER_EVENT][SAMPLE_BUFFER_SIZE];

    uint16_t sample_index;
    uint16_t last_sample_index;
    uint16_t sample_size_before;

    int32_t  record_id;
    uint32_t last_found_record_time;
    uint32_t file_crc;

    uint16_t flash_address;
    uint16_t sample_count;

    uint16_t acc_overrrun_count;

    int16_t record_num;
    uint8_t record_reason;

    bool accident_stage;
    bool accident_identified;
    bool accident_saving;

    uint8_t last_sent_record_num;
    uint8_t last_sent_record_num_count;

} AccRecord;

void     record_init(void);
uint8_t  record_scan_for_new_records(bool forced);
void     record_trigger(uint8_t reason);
void     record_write_status(uint8_t record_num, uint8_t indication_idx, uint8_t value);
void     record_print(unsigned char record_num);
int16_t  record_search(uint32_t record_id);
uint32_t GetRandomNumber(void);

void recorder_task(void *pvParameter);
void close_recording(void);
void DeleteRecord(uint8_t record_num);
unsigned char check_stuck_record(unsigned char record_num);
void SendRecordAlert(uint32_t record_id);