/*
 * bootloader.h
 *
 *  Created on: Feb 20, 2024
 *      Author: minh
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_
//version 1.0
#define BL_VERSION 0x10

#define INVALID_SECTOR 0x04

/*Some Start and End addresses of different memories of STM32F446xx MCU */
/*Change this according to your MCU */
#define SRAM1_SIZE            128*1024     // STM32F411V6 has 128KB of SRAM1
#define SRAM1_END             (SRAM1_BASE + SRAM1_SIZE)
#define FLASH_SIZE            512*1024     // STM32F411V6 has 512KB of FLASH

// our bootloader commands

//#define  <command name >	<command_code>

typedef enum {
	BL_GET_VER = 0x51,			//read the bootloader version from the MCU
	BL_GET_HELP,				//know what are the commands supported by the bootloader
	BL_GET_CID,					//read the MCU chip identification number
	BL_GET_RDP_STATUS,			//read the FLASH Read Protection level
	BL_GO_TO_ADDR,				//jump bootloader to specified address
	BL_FLASH_ERASE,				//mass erase or sector erase of the user flash
	BL_MEM_WRITE,				//write data in to different memories of the MCU
	BL_EN_RW_PROTECT,			//enable or disable read/write protect on different sectors of the user flash
	BL_MEM_READ,				//read data from different memories of the microcontroller
	BL_READ_SECTOR_P_STATUS,	//read all the sector protection status
	BL_OTP_READ,				//read the OTP contents
	BL_DIS_R_W_PROTECT			//disable all sector read/write protection
}bl_support_command_t;

/* ACK and NACK bytes*/
typedef enum {
	BL_ACK = 0XA5,
	BL_NACK = 0x7F
}bl_return_code_t;

/*CRC*/
typedef enum {
	VERIFY_CRC_SUCCESS = 0,
	VERIFY_CRC_SUCCESS
}crc_status_code_t;

typedef enum {
	ADDR_VALID = 0,
	ADDR_INVALID
}address_status_code_t;

uint8_t supported_commands[] = {
                               BL_GET_VER ,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               BL_READ_SECTOR_P_STATUS} ;


#endif /* INC_BOOTLOADER_H_ */
