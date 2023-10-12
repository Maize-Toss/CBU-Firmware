/*
 * reader.h
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */

#ifndef INC_READER_H_
#define INC_READER_H_

#include <stdbool.h>

#define ARR_SIZE;

void calculateRawScores(uint8_t* teamRawScore, bool isTeam1);

typedef struct RFID_interface
{
	SPI_HandleTypeDef* hspi;
	int cs;
	// define characteristics
	bool isSideA; // sideA or sideB of the court
} RFID_interface;

typedef struct BeanBag_interface {
	uint32_t uid;
} BeanBag_interface;

// global variable for RFID tag info to be tracked
BeanBag_interface BagInfo[8];

// global variable for tracking the status of each bag
uint8_t BagStatus[8] = {0,0,0,0,0,0,0,0};

// used to set up struct and initialize colors, tag IDs, etc
// modifies beanbag_interface
// returns nothing
void BeanBag_setup();

// find ID in array of BeanBag_interfaces
// modifies nothing
// returns index in array if found or -1 if not found
int BeanBag_findIDinArray(const int id);

// initialize passed RFID reader
// modifies RFID_interface
// returns 1 if succeeded, 0 otherwise
int RFID_init(RFID_interface *pInterface);

// performs a blocking or nonblocking read of the given RFID reader
// modifies buf to hold an array of IDs detected by the reader
// returns number of IDs read
int RFID_read(const RFID_interface *pInterface, int *buf, int timeout_ms, bool blocking);

// reads all RFID antenna regions and updates the global BagStatus array
void RFID_readArray(void);

#endif /* INC_READER_H_ */
