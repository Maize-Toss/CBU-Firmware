/*
 * reader.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "reader.h"
#include "queue.h"

#define READ_LENGTH 2000 // 50 ms

// global variable for RFID tag info to be tracked
//0x0050510800505100
BeanBag_interface BagInfo[NUM_BAGS] = {
		{{0xE0040150B8F856E2, 0xE0040150B8F854BB, 0xE0040150B8F854DB, 0xE0040150B8F74A4F}, false, -1}, // Red 1
		{{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000}, false, -1}, // Red 2
		{{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000}, false, -1}, // Red 3
		{{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000}, false, -1}, // Red 4
		{{0xE0040150B8F819C7, 0xE0040150B8F85760, 0xE0040150B8F8173A, 0xE0040150B8F8545C}, false, -1}, // Blue 1
		{{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0xE0040150B8F85FFF}, false, -1}, // Blue 2
		{{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000}, false, -1}, // Blue 3
		{{0x0000000000000000, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000}, false, -1}, // Blue 4
};

const bool ANT_ENABLED[12] = {1,1,1,1, 1,1,1,1, 1,1,1,1};

// global variable for tracking the status of each bag
// Red (Team 1): 0-3; Blue (Team 2): 4-7
uint8_t BagStatus[NUM_BAGS] = {0,0,0,0,0,0,0,0};

// map of antenna number to point value
const uint8_t ANT_POINTS_MAP[12] = {1,1,1,1, 1,1,1,1, 1,1,1,1};

extern uint8_t select_rfid_channel(uint8_t channel_index);

static const uint8_t UID_ZERO_CMP[UID_SIZE_15693]; // auto-initialized to zero TODO test lol

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
	static int errorCounter = 0;

	BeanBag_clearDetected();
	BeanBag_clearScore();

	RFIDEvent_t xRFIDEvent;

	for(uint8_t ant_number = 5; ant_number <= 5; ant_number++){
		if (!ANT_ENABLED[ant_number-1]) continue; // ANT not plugged in, don't waste your time

		select_rfid_channel(ant_number);

		vTaskDelay(10);

		for(;;) {
		    /* Wait until there is something to do. */
//			BaseType_t ret = xQueueReceive( xRFIDEventQueueHandle, &xRFIDEvent, osWaitForever );
//			if (ret != pdTRUE) {
//				errorCounter++;
//			}

			if (handler->irq_flag) {

				memset(handler->uid, 0, sizeof(handler->uid)); // clear uid

				st25r95_service(handler);

				// only do this if anticollision is disabled
				// since anticollision handles this already
				if (!ANTICOL_15693 && memcmp(handler->uid, UID_ZERO_CMP, sizeof(handler->uid)) ) { // check for non-zero uid, returns 0 if same
					int bag_number = BeanBag_findIDinArray(handler);

					if (bag_number == -1) continue; // UID not found
					else if (bag_number == -2) continue; // Bag already scanned

					BagStatus[bag_number] = ANT_POINTS_MAP[ant_number - 1];

				}
			}

			if (handler->timeout_flag) {
				handler->timeout_flag = 0;

				HAL_GPIO_WritePin(GPIOB, 1 << 4, 0);

				osTimerStop(RFIDTimeoutHandle);
				osTimerStart(RFIDTimeoutHandle, pdMS_TO_TICKS(TIMER_PERIOD_MS));

				break;
			}

//		    /* Perform a different action for each event type. */
//		    if (xRFIDEvent == EVENT_READ) {
//		    	memset(handler->uid, 0, sizeof(handler->uid)); // clear uid
//
//		    	st25r95_service(handler);
//
//		    	//xQueueReset(xRFIDEventQueueHandle);
//
//				if ( memcmp(handler->uid, UID_ZERO_CMP, sizeof(handler->uid)) ) { // check for non-zero uid, returns 0 if same
//					int bag_number = BeanBag_findIDinArray(handler);
//
//					if (bag_number == -1) continue; // UID not found
//					else if (bag_number == -2) continue; // Bag already scanned
//
//					BagStatus[bag_number] = ANT_POINTS_MAP[ant_number - 1];
//				}
//		    } else if (xRFIDEvent == EVENT_TIMEOUT) {
//		    	xQueueReset(xRFIDEventQueueHandle);
//
//		    	HAL_GPIO_WritePin(GPIOB, 1 << 4, 0);
//
//		    	osTimerStop(RFIDTimeoutHandle);
//		    	osTimerStart(RFIDTimeoutHandle, pdMS_TO_TICKS(TIMER_PERIOD_MS));
//
//		    	break;
//		    }
		}
	}
}

void BeanBag_setup(void) {

	BeanBag_clearDetected();

}

void BeanBag_clearDetected(void) {

	for (int i = 0; i < NUM_BAGS; ++i) {
		BagInfo[i].detected = false;
		BagInfo[i].ant_channel = -1;
	}
}

void BeanBag_clearScore(void) {

	for (int i = 0; i < NUM_BAGS; ++i) {
		BagStatus[i] = 0;
	}
}

int BeanBag_findIDinArray(st25r95_handle *handler) {

	uint64_t uid = 0;

	memcpy(&uid, handler->uid, UID_SIZE_15693);

	int bag_number = -1;

	for (int i = 0; i < NUM_BAGS; ++i) {
		if (uid == BagInfo[i].uid[0] || uid == BagInfo[i].uid[1] || uid == BagInfo[i].uid[2] || uid == BagInfo[i].uid[3]) {
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
		BagInfo[bag_number].ant_channel = handler->ant_channel;
		return bag_number;
	}

}
