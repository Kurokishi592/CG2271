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



}


void ledOn(TLED led) {
	switch(led) {
	case RED:
		// Code to turn on RED LED
		break;

	case GREEN:
		// Code to turn on GREEN LED
		break;

	case BLUE:
		// Code to turn on BLUE LED
		break;
	}
}

void ledOff(TLED led) {
	switch(led) {
	case RED:
		// Turn off RED led here
		break;

	case GREEN:
		// Turn off GREEN led here
		break;

	case BLUE:
		// Turn off BLUE led here
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
    	if(// Code to read switch) {

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
        	count = (count + 1) % 6;

    	}

    }
    return 0 ;
}

