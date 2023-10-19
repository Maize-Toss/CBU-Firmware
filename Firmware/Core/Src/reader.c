/*
 * reader.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "reader.h"

// global variable for RFID tag info to be tracked
BeanBag_interface BagInfo[8];

// global variable for tracking the status of each bag
uint8_t BagStatus[8] = {0,0,0,0,0,0,0,0};

void calculateRawScore(uint8_t* teamRawScore, bool isTeam1) {

	uint8_t* bag_p = BagStatus;

	if (isTeam1) bag_p +=4;

	*teamRawScore = 0;

	// Team 0 routine
	for (int i = 0; i < 4; ++i) {
		*teamRawScore += bag_p[i];
	}

}

// reads all RFID antenna regions and updates the global BagStatus array
void RFID_readArray(void) {

}
