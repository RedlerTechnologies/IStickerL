#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BFT_PACKET_DATA_SIZE 16
#define BFT_PACKET_SIZE (BFT_PACKET_DATA_SIZE + 4)

// file transfer test simulation
//#define SIMULATE_TRANSFER
#define SIMULATE_WITH_POSITIVE_ACK
#define FILE_DELAY 100

#define MAX_FILE_REGIONS 4

#define FILE_TYPE_PARAMS 0
#define FILE_TYPE_RECORD 100

typedef struct {
    // table values in multiply of 256
    uint16_t reg_start_address[MAX_FILE_REGIONS];
    uint16_t reg_size[MAX_FILE_REGIONS];

    uint32_t file_length;
    uint32_t crc;
    uint32_t prev_crc;
    uint32_t last_packet_crc;
    uint32_t last_ack_time;
    int32_t  next_read_address;
    int32_t  prev_packet_address;
    uint32_t record_id;

    int16_t record_num;
    uint8_t offset;
    uint8_t state;
    uint8_t retries;
    uint8_t num_of_regions;
    uint8_t file_type;
} BleReadingFileState;

void BFT_start(unsigned short record_num, unsigned char is_record);
void BFT_send_next_packet(void);
void FileTransferFailed(bool abort);
bool in_sending_file(void);