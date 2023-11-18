/*
 * bluetooth.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "bluetooth.h"
#define MAX_JSON_BYTE_SIZE 128



void readyToRead( BLE_interface* ble, uint8_t* rx_buff, uint32_t size){
	HAL_UART_Receive(ble->huart, rx_buff, size, 10000);// Sending in normal mode
}

uint32_t broadcast(uint8_t* tx_buff, uint32_t size, BLE_interface* ble){
	HAL_UART_Transmit(ble->huart, tx_buff, size, 10000);// Sending in normal mode
}


