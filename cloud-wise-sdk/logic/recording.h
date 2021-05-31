#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "tracking_algorithm.h"

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

typedef enum {
    ACC_RECORD_START = 0,
    ACC_RECORD_IDENTIFIED,
    ACC_RECORD_CONTINUE,
} AccRecordState;

typedef struct {
    uint8_t samples1[3 * 2 * RECORD_BUFFER_SAMPLE_SIZE];
    uint8_t samples2[RECORD_HEADER_SIZE + 3 * 2 * RECORD_BUFFER_SAMPLE_SIZE];

    int32_t  record_id;
    uint32_t last_found_record_time;
    uint32_t file_crc;

    uint16_t sample_index;
    uint16_t flash_address;
    uint16_t sample_count;

    int16_t record_num;
    uint8_t buffer_stage;
    uint8_t state;
    uint8_t record_reason;

} AccRecord;


void record_init(void);
uint8_t record_scan_for_new_records(bool forced);
void record_trigger(uint8_t reason);
bool record_is_active(void);
void record_write_status(uint8_t record_num, uint8_t indication_idx, uint8_t value);
void record_add_sample(AccConvertedSample *acc_sample);
void record_print(unsigned char record_num);
int16_t record_search(uint32_t record_id);
uint32_t GetRandomNumber(void);