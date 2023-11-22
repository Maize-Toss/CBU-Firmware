/*
 * reader.h
 *
 *  Created on: Oct 11, 2023
 *      Author: evanm
 */

#ifndef INC_READER_H_
#define INC_READER_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "st25r95.h"

#include "cmsis_os.h"

#define NUM_BAGS 8

#define SEL0_GPIO_Port GPIOC
#define SEL1_GPIO_Port GPIOC
#define SEL2_GPIO_Port GPIOB
#define SEL3_GPIO_Port GPIOB
#define SEL4_GPIO_Port GPIOC
#define SEL5_GPIO_Port GPIOC
#define SEL6_GPIO_Port GPIOB
#define SEL7_GPIO_Port GPIOB

#define SEL0_Pin GPIO_PIN_2
#define SEL1_Pin GPIO_PIN_1
#define SEL2_Pin GPIO_PIN_15
#define SEL3_Pin GPIO_PIN_14
#define SEL4_Pin GPIO_PIN_7
#define SEL5_Pin GPIO_PIN_6
#define SEL6_Pin GPIO_PIN_13
#define SEL7_Pin GPIO_PIN_12

void calculateRawScore(uint8_t* teamRawScore, bool isBlue);

typedef struct RFID_interface
{
	SPI_HandleTypeDef* hspi;
	int cs;
	// define characteristics
	bool isSideA; // sideA or sideB of the court
} RFID_interface;

typedef struct BeanBag_interface {
	uint64_t uid1;
	uint64_t uid2;
	uint64_t uid3;
	uint64_t uid4;

	bool detected;
} BeanBag_interface;

// global variable for RFID tag info to be tracked
extern BeanBag_interface BagInfo[NUM_BAGS];

// global variable for tracking the status of each bag
extern uint8_t BagStatus[NUM_BAGS];

// used to set up struct and initialize colors, tag IDs, etc
// modifies beanbag_interface
// returns nothing
void BeanBag_setup();

// Clears detected variable for all beanbags
void BeanBag_clearDetected(void);

// find ID in array of BeanBag_interfaces
// modifies nothing
// returns index (bag number) in array if found or -1 if not found
// returns -2 if bag already detected
int BeanBag_findIDinArray(st25r95_handle *handler);

// initialize passed RFID reader
// modifies RFID_interface
// returns 1 if succeeded, 0 otherwise
int RFID_init(RFID_interface *pInterface);

// performs a blocking or nonblocking read of the given RFID reader
// modifies buf to hold an array of IDs detected by the reader
// returns number of IDs read
int RFID_read(const RFID_interface *pInterface, int *buf, int timeout_ms, bool blocking);

// reads all RFID antenna regions and updates the global BagStatus array
void RFID_readArray(st25r95_handle *handler);

#endif /* INC_READER_H_ */
