/*
 * bluetooth.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "bluetooth.h"

// initializes BLE interface
void initBLE(BLE_interface * ble) {

}

void setDeviceName(char* name, BLE_interface* ble) {

}

// check if device is connected to target
bool isConnected(char* target, BLE_interface* ble) {
return true;
}

// perform any maintenance to connect and disconnect to target
// return status
uint32_t connect(char* target, BLE_interface* ble) {
return 1;
}

uint32_t disconnect(char* target, BLE_interface* ble) {
return 1;
}

// poll for events
void readBLE(uint8_t* rx_buff, uint32_t* size, BLE_interface* ble) {

}

// broadcast payload to other ble modules
// return status
uint32_t broadcast(uint8_t* tx_buffer, uint32_t size, BLE_interface* ble) {
return 1;
}
