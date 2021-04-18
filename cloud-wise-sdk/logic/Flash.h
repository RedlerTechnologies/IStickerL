#pragma once

#include "hal/hal_data_types.h"


/* Flash ID */
#define GD25Q32_FLASH_ID 		0xC8

/* Status Register Bits */
#define STATUS_BUSY				0x01
#define	STATUS_WEL				0x02
#define	STATUS_BP0				0x04
#define	STATUS_BP1				0x08
#define	STATUS_TB   			0x20
#define	STATUS_SRP				0x80

/* Flash operation command */
#define WRITE_STATUS_REG		0x01
#define PAGE_PROGRAM			0x02
#define READ_DATA_BYTE			0x03
#define READ_STATUS_REG			0x05
#define CHIP_EARSE				0x60
#define READ_ID					0x9f