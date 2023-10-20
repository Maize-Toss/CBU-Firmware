/*
 * bluetooth.h
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

//#define SLEEP_MODE "AT+SLEEP"
//#define AT "AT"
//#define QUERY_ADDR "AT+ADDR?"
//#define QUERY_BAUD "AT+BAUD?"
//#define CONNECT_TO_LAST "AT+CONNL"
//#define CONNECT_TO_ADDRS "AT+CO"
//#define ACCEPTED_REQUEST "OK+CONNA"
//#define CONNECTED "OK+CONN"
//#define CONNECTION_ERROR "OK+CONNE"
//#define CONNECTION_FAILED "OK+CONNF"
//#define DISCOVERY_SCAN "AT+DISC?"
//#define QUERY_MODE "AT+MODE?"
//#define QUERY_NAME "AT+NAME?"
//#define QUERY_POWER "AT+PCTL"
//#define SAVE_ADDRS "AT+SAVE?"
//#define SHOW_NAME  "AT+SHOW"


typedef struct BLE_interface
{
	UART_HandleTypeDef *huart;
	GPIO_TypeDef *cs_base;

	uint16_t cs_pin;
	char* name;
	osMutexId_t *mutex;

	uint8_t addr[6]; // bluetooth addr is 6 bytes, colon separated
} BLE_interface;


// poll for events
Void readyToRead(BLE_interface* ble, uint8_t* rx_buff);

// broadcast payload to other ble modules
// return status
uint32_t broadcast(uint32_t* tx_buffer, uint32_t size, BLE_interface* ble);


#endif /* INC_BLUETOOTH_H_ */
