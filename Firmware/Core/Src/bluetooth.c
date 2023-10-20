/*
 * bluetooth.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "bluetooth.h"
#define MAX_JSON_BYTE_SIZE 128



Void readBLE(uint32_t timeout, BLE_interface* ble, uint8_t* rx_buff){
	HAL_UART_Receive(ble->huart, rx_buff, sizeof(rx_buff), MAX_JSON_BYTE_SIZE);// Sending in normal mode
}

uint32_t broadcast(uint32_t* tx_buff, uint32_t size, BLE_interface* ble){
	HAL_UART_Transmit(ble->huart, tx_buff, size,10);// Sending in normal mode
}
