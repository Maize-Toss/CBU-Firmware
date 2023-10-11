/*
 * reader.c
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */
#include "reader.h"

void calculateRawScores(uint8_t* teamRawScore, bool isTeam1) {

	uint8_t* bag_p = BagStatus;

	if (isTeam1) bag_p +=4;

	*teamRawScore = 0;

	// Team 0 routine
	for (int i = 0; i < 4; ++i) {
		*teamRawScore += bag_p[i];
	}

}
