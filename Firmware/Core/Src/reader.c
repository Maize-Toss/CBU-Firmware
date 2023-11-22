/*
 * reader.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "reader.h"

#define READ_LENGTH 50 // 50 ms

// global variable for RFID tag info to be tracked
BeanBag_interface BagInfo[NUM_BAGS] = {
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Red 1
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Red 2
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Red 3
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Red 4
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Blue 1
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Blue 2
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Blue 3
		{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000, false}, // Blue 4
};

const bool ANT_ENABLED[12] = {1,1,1,1, 0,0,0,0, 1,1,1,1};

// global variable for tracking the status of each bag
// Red (Team 1): 0-3; Blue (Team 2): 4-7
uint8_t BagStatus[NUM_BAGS] = {0,0,0,0,0,0,0,0};

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

	BeanBag_clearDetected();

	for(uint8_t ant_number = 1; ant_number < 12; ant_number++){
		if (!ANT_ENABLED[ant_number-1]) continue; // ANT not plugged in, don't waste your time

		st25r95_off(handler);
		select_rfid_channel(ant_number);

		TickType_t startTick = xTaskGetTickCount();

		st25r95_idle(handler);
		while ( xTaskGetTickCount() < startTick + pdMS_TO_TICKS(READ_LENGTH) ) {
			memset(handler->uid, 0, sizeof(handler->uid)); // clear uid
			st25r95_service(handler);

			if ( memcmp(handler->uid, UID_ZERO_CMP, sizeof(handler->uid)) ) { // non-zero uid
				int bag_number = BeanBag_findIDinArray(handler);

				if (bag_number == -1) continue; // UID not found
				else if (bag_number == -2) continue; // Bag already scanned

				BagStatus[bag_number] = ANT_POINTS_MAP[ant_number - 1];
			}
		}
	}
}

void BeanBag_setup(void) {

	BeanBag_clearDetected();

}

void BeanBag_clearDetected(void) {

	for (int i = 0; i < NUM_BAGS; ++i) {
		BagInfo[i].detected = false;
	}
}

int BeanBag_findIDinArray(st25r95_handle *handler) {

	uint64_t uid = 0;

	memcpy(&uid, handler->uid, 8);

	int bag_number = -1;

	for (int i = 0; i < NUM_BAGS; ++i) {
		if (uid == BagInfo[i].uid1 || uid == BagInfo[i].uid2 || uid == BagInfo[i].uid3 || uid == BagInfo[i].uid4) {
			bag_number = i;
			break;
		}
	}

	if (bag_number == -1) { // UID not mapped to a bag
		return -1;

	} else if (BagInfo[bag_number].detected) { // return bag already detected status code
		return -2;

	} else { // valid UID, not already detected
		BagInfo[bag_number].detected = true;
		return bag_number;
	}

}
