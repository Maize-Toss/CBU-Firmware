/*
 * bluetooth.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "bluetooth.h"
#define MAX_JSON_BYTE_SIZE 128



void readyToRead( BLE_interface* ble, uint8_t* rx_buff){
	HAL_UART_Receive_IT(ble->huart, rx_buff, sizeof(rx_buff));// Sending in normal mode
}

uint32_t broadcast(uint8_t* tx_buff, uint32_t size, BLE_interface* ble){
	HAL_UART_Transmit_IT(ble->huart, tx_buff, size);// Sending in normal mode
}


