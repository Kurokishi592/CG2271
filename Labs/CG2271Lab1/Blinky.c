	/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Blinky.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

#define SWITCH_PIN	4	// PTA4

typedef enum tl {
	RED, GREEN, BLUE
} TLED;



/*
Usage Example:
delay(0x80000);
*/

void initGPIO() {
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);
	PORTE->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_PIN] |= PORT_PCR_MUX(1);
	PORTE->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[BLUE_PIN] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(1);
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_MUX(1);

	GPIOA->PDDR &= ~(1 << SWITCH_PIN);
	GPIOE->PDDR |= (1 << RED_PIN | 1 << BLUE_PIN);
	GPIOD->PDDR |= (1 << GREEN_PIN);
}


void ledOn(TLED led) {
	switch(led) {
	case RED:
		// Code to turn on RED LED
		GPIOE->PCOR |= (1 << RED_PIN);
		break;

	case GREEN:
		// Code to turn on GREEN LED
		GPIOD->PCOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		// Code to turn on BLUE LED
		GPIOE->PCOR |= (1 << BLUE_PIN);
		break;
	}
}

void ledOff(TLED led) {
	switch(led) {
	case RED:
		// Turn off RED led here
		GPIOE->PSOR |= (1 << RED_PIN);
		break;

	case GREEN:
		// Turn off GREEN led here
		GPIOD->PSOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		// Turn off BLUE led here
		GPIOE->PSOR |= (1 << BLUE_PIN);
		break;
	}
}

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Hello World\r\n");

    /* Force the counter to be placed into memory. */
    /* Enter an infinite loop, just incrementing a counter. */
    initGPIO();
    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);

    int count = 0;
    while(1) {
    	if(!(GPIOA->PDIR & (0b1 << SWITCH_PIN))) {
        	switch(count) {
        	case 0:
        		ledOn(RED);
        		break;
        	case 1:
        		ledOn(GREEN);
        		break;
        	case 2:
        		ledOn(BLUE);
        		break;
        	case 3:
        		ledOff(BLUE);
        		break;
        	case 4:
        		ledOff(GREEN);
        		break;
        	case 5:
        		ledOff(RED);
        		break;

        	default:
        		count=0;

        	}
    	}
        	count = (count + 1) % 6;
    }
    return 0 ;

}

