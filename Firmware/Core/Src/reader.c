/*
 * reader.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "reader.h"
#include "stm32l4xx_hal.h"

#define SEL_PORT GPIOB

const uint16_t sel[8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};

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

	for(uint8_t row = 0; row < 3; row++){
		// iterate through sel6 and sel7
	    HAL_GPIO_WritePin(SEL_PORT, sel[6], (row >> 1) & 0b01);
	    HAL_GPIO_WritePin(SEL_PORT, sel[7], (row & 0b01) );

		for(uint8_t column = 0; column < 4; column++){
			// iterate through each element in the row
			HAL_GPIO_WritePin(SEL_PORT, sel[row*2], (column >> 1) & 0b01);
			HAL_GPIO_WritePin(SEL_PORT, sel[row*2 + 1], (column & 0b01) );

			// Now read from selected antenna

		}
	}
}
