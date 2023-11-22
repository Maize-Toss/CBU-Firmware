/*
 * reader.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "reader.h"
#include "stm32l4xx_hal.h"
#include "st25r95.h"

#define READ_LENGTH 50 // 50 ms

// global variable for RFID tag info to be tracked
BeanBag_interface BagInfo[8];

// global variable for tracking the status of each bag
// Red (Team 1): 0-3; Blue (Team 2): 4-7
uint8_t BagStatus[8] = {0,0,0,0,0,0,0,0};

// map of antenna number to point value
const uint8_t ANT_POINTS_MAP[12] = {1,1,1,1, 3,3,3,3, 1,1,1,1};

extern uint8_t select_rfid_channel(uint8_t channel_index);

static const uint8_t UID_ZERO_CMP[UID_SIZE]; // auto-initialized to zero TODO test lol

void calculateRawScore(uint8_t* teamRawScore, bool isBlue) {

	uint8_t* bag_p = BagStatus;

	// Shift pointer to blue section of array
	if (isBlue) bag_p += 4;

	*teamRawScore = 0;

	// Score count routine
	for (int i = 0; i < 4; ++i) {
		*teamRawScore += bag_p[i];
	}

}

// reads all RFID antenna regions and updates the global BagStatus array
void RFID_readArray(st25r95_handle *handler) {

	clearBagsDetected();

	for(uint8_t ant_number = 1; ant_number < 12; ant_number++){
		st25r95_off(handler);
		select_rfid_channel(ant_number);

		st25r95_idle(handler);
		while ( xTaskGetTickCount() < startTick + pdMS_TO_TICKS(READ_LENGTH) ) {
			memset(handler->uid, 0, sizeof(handler->uid)); // clear uid
			st25r95_service(handler);

			if ( memcmp(handler->uid, UID_ZERO_CMP, sizeof(handler->uid)) ) { // non-zero uid
				int bag_number = getBagNumber(handler);

				if (bag_number == -1) continue; // UID not found
				else if (bag_number == -2) continue; // Bag already scanned

				BagStatus[bag_number] = ANT_POINTS_MAP[ant_number - 1];
			}

		}

	}

}

void BeanBag_setup(void) {

}

void BeanBag_clearDetected(void) {



}

int BeanBag_findIDinArray(st25r95_handle *handler) {



}
