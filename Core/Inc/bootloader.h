/*
 * bootloader.h
 *
 *  Created on: Feb 20, 2024
 *      Author: minh
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define INVALID_SECTOR 0x04

/*Some Start and End addresses of different memories of STM32F446xx MCU */
/*Change this according to your MCU */
#define SRAM1_SIZE            128*1024     // STM32F411V6 has 128KB of SRAM1
#define SRAM1_END             (SRAM1_BASE + SRAM1_SIZE)
#define FLASH_SIZE            512*1024     // STM32F411V6 has 512KB of FLASH
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U
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
	VERIFY_CRC_FAIL
}crc_status_code_t;

typedef enum {
	ADDR_VALID = 0,
	ADDR_INVALID
}address_status_code_t;


void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_getcid_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_getrdp_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_go_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_flash_erase_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_mem_write_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_en_rw_protect(uint8_t *bl_rx_buffer);
void bootloader_handle_mem_read (uint8_t *bl_rx_buffer);
void bootloader_handle_read_sector_protection_status(uint8_t *bl_rx_buffer);
void bootloader_handle_read_otp(uint8_t *bl_rx_buffer);
void bootloader_handle_dis_rw_protect(uint8_t *bl_rx_buffer);

void bootloader_send_ack(uint8_t command_code);
void bootloader_send_nack(void);

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host);
uint8_t get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *bl_tx_buffer,uint32_t len);

uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);

uint16_t read_OB_rw_protection_status(void);

#endif /* INC_BOOTLOADER_H_ */
