/*
 * bootloader.c
 *
 *  Created on: Feb 20, 2024
 *      Author: minh
 */

#include "bootloader.h"
#include "debug_console.h"

#define BL_RX_LEN  200
static uint8_t bl_rx_buffer[BL_RX_LEN];
static uint8_t bootloader_version = 10;

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;

static uint8_t supported_commands[] = {	BL_GET_VER ,
										BL_GET_HELP,
										BL_GET_CID,
										BL_GET_RDP_STATUS,
										BL_GO_TO_ADDR,
										BL_FLASH_ERASE,
										BL_MEM_WRITE,
										BL_READ_SECTOR_P_STATUS};

void  bootloader_uart_read_data(void) {
	uint8_t command_length = 0;
	while(1) {
		memset(bl_rx_buffer, 0, sizeof(bl_rx_buffer));

		// Read length to follow
		HAL_UART_Receive(&huart2, bl_rx_buffer, 1, HAL_MAX_DELAY);
		command_length = bl_rx_buffer[0];
		HAL_UART_Receive(&huart2, &bl_rx_buffer[1], command_length, HAL_MAX_DELAY);

		// Compare CRC
		uint32_t command_packet_len = bl_rx_buffer[0] + 1 ;
		uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));
		if (bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc) == VERIFY_CRC_FAIL)
		{
			printmsg("DEBUG: Checksum fail !!\n");
			//checksum is wrong send nack
			bootloader_send_nack();
			return;
		}

		// Send ACK
		printmsg("DEBUG: Checksum success !!\n");
		bootloader_send_ack(pBuffer[0]);

		// Send more information
		switch(bl_rx_buffer[1]) {
		case BL_GET_VER:
			bootloader_handle_getver_cmd(bl_rx_buffer);
			break;
		case BL_GET_HELP:
			bootloader_handle_gethelp_cmd(bl_rx_buffer);
			break;
		case BL_GET_CID:
			bootloader_handle_getcid_cmd(bl_rx_buffer);
			break;
		case BL_GET_RDP_STATUS:
			bootloader_handle_getrdp_cmd(bl_rx_buffer);
			break;
		case BL_GO_TO_ADDR:
			bootloader_handle_go_cmd(bl_rx_buffer);
			break;
		case BL_FLASH_ERASE:
			bootloader_handle_flash_erase_cmd(bl_rx_buffer);
			break;
		case BL_MEM_WRITE:
			bootloader_handle_mem_write_cmd(bl_rx_buffer);
			break;
		case BL_EN_RW_PROTECT:
			bootloader_handle_en_rw_protect(bl_rx_buffer);
			break;
		case BL_MEM_READ:
			bootloader_handle_mem_read(bl_rx_buffer);
			break;
		case BL_READ_SECTOR_P_STATUS:
			bootloader_handle_read_sector_protection_status(bl_rx_buffer);
			break;
		case BL_OTP_READ:
			bootloader_handle_read_otp(bl_rx_buffer);
			break;
		case BL_DIS_R_W_PROTECT:
			bootloader_handle_dis_rw_protect(bl_rx_buffer);
			break;
		default:
			printmsg("BL_DEBUG_MSG:Invalid command code received from host \n");
			break;
		}
	}
}

void bootloader_jump_to_user_app(void) {
	void (*app_reset_handler)(void);
	printmsg("DEBUG: bootloader_jump_to_user_app");

	// 1. configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("DEBUG: MSP value : %#x\n",msp_value);

	//This function comes from CMSIS.
	__set_MSP(msp_value);

	/* 2. Now fetch the reset handler address of the user application
	 * from the location FLASH_SECTOR2_BASE_ADDRESS+4
	 */
	uint32_t reset_handler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);
	app_reset_handler = (void*) reset_handler_address;
	printmsg("DEBUG: App reset handler at address : %#x\n", app_reset_handler);

	//3. jump to reset handler of the user application
	app_reset_handler();
}

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer) {
	uint8_t bl_version;
	printmsg("DEBUG: bootloader_handle_getver_cmd\n");
	bl_version = get_bootloader_version();
	printmsg("DEBUG: BL_VER : %d %#x\n", bl_version, bl_version);
}

void bootloader_handle_gethelp_cmd(uint8_t *bl_rx_buffer) {
	printmsg("DEBUG: bootloader_handle_gethelp_cmd\n");
	bootloader_uart_write_data(supported_commands, sizeof(supported_commands));
}

void bootloader_handle_getcid_cmd(uint8_t *bl_rx_buffer) {
	uint16_t bl_cid_num = 0;
	printmsg("DEBUG: bootloader_handle_getcid_cmd\n");
	bl_cid_num = get_mcu_chip_id();
	printmsg("DEBUG: MCU id : %d %#x !!\n",bl_cid_num, bl_cid_num);
	bootloader_uart_write_data((uint8_t *)&bl_cid_num, sizeof(bl_cid_num));
}

void bootloader_handle_getrdp_cmd(uint8_t *bl_rx_buffer) {

}

void bootloader_handle_go_cmd(uint8_t *bl_rx_buffer) {


}

void bootloader_handle_flash_erase_cmd(uint8_t *bl_rx_buffer) {

}

void bootloader_handle_mem_write_cmd(uint8_t *bl_rx_buffer) {

}

void bootloader_handle_en_rw_protect(uint8_t *pBuffer) {

}

void bootloader_handle_mem_read (uint8_t *pBuffer) {

}

void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer) {

}

void bootloader_handle_read_otp(uint8_t *pBuffer) {

}

void bootloader_handle_dis_rw_protect(uint8_t *pBuffer) {

}

//Bootloader return status for command
void bootloader_send_ack(uint8_t command_code)
{
	 //here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	uint8_t follow_len = 0;
	if(command_code == BL_GET_VER) {
		follow_len = 1;
	}
	switch (command_code) {
	case BL_GET_VER:
		follow_len = 1;
		break;
	case BL_GET_HELP:
		follow_len = sizeof(supported_commands);
		break;
	default:
		break;
	}
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(&huart2, ack_buf, 2, HAL_MAX_DELAY);

}

void bootloader_send_nack(void) {
	uint8_t ack_buf;
	ack_buf = BL_NACK;
	HAL_UART_Transmit(&huart2, &ack_buf, 1, HAL_MAX_DELAY);
}

crc_status_code_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host) {
	uint32_t uwCRCValue = 0xff;
	for (uint32_t i = 0 ; i < len ; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}
	 /* Reset CRC Calculation Unit */
	__HAL_CRC_DR_RESET(&hcrc);
	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}
	return VERIFY_CRC_FAIL;
}

uint8_t get_bootloader_version(void) {
	return bootloader_version;
}

void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len) {
	HAL_UART_Transmit(&huart2, pBuffer, len, HAL_MAX_DELAY);
}

uint16_t get_mcu_chip_id(void) {
	/*
	The STM32Fxx MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus. This code is accessible using the
	JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  cid;
}

uint8_t get_flash_rdp_level(void) {
	return 0;
}

uint8_t verify_address(uint32_t go_address) {
	return 0;
}

uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector) {
	return 0;
}

uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len) {
	return 0;
}

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable) {
	return 0;
}

uint16_t read_OB_rw_protection_status(void) {
	return 0;
}
