/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271Lab2Soln.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/*
 * @brief   Application entry point.
 */

// LEDs
// RED: PTE31
// GREEN: PTD5
// BLUE: PTE29

#define REDLED		31
#define GREENLED	5
#define BLUELED		29

void initLEDs() {
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);
	PORTE->PCR[REDLED] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[REDLED] |= PORT_PCR_MUX(1);

	PORTE->PCR[BLUELED] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[BLUELED] |= PORT_PCR_MUX(1);

	PORTD->PCR[GREENLED] &= PORT_PCR_MUX_MASK;
	PORTD->PCR[GREENLED] |= PORT_PCR_MUX(1);

	// Set PDDR
	GPIOE->PDDR |= ((1 << REDLED) | (1 << BLUELED));
	GPIOD->PDDR |= (1 << GREENLED);
}

// Switch off all LEDs
void ledOff() {
	GPIOE->PSOR |= ((1 << REDLED) | (1 << BLUELED));
	GPIOD->PSOR |= (1 << GREENLED);
}

// Switch on all LEDs
void ledOn() {
	GPIOE->PCOR |= ((1 << REDLED) | (1 << BLUELED));
	GPIOD->PCOR |= (1 << GREENLED);
}
/* Initialize interrupts for SW2 and SW3 */

// SW2 is connected to PTC3, SW3 to PTA4

#define SW2		3
#define SW3		4

// SW2 is connected to PTC3
void initSW2Interrupt() {

    // Fill in code to initialize SW2 interrupt
}

// SW3 is connected to pin PTA4
void initSW3Interrupt() {

    // Fill in code to initialize SW3 interrupt
}



int mode = 0;

// SW3 handler. SW3 toggles the mode
void /*fill appropriate handler name here*/ {
	// Fill instruction to clear pending IRQ here

	// Check that SW3 was triggered
	if(// Fill appropriate statement here) {
		// mode cycles from 0 to 1 to 2 to 0, etc.
		mode = (mode + 1) % 3;
	}

    // Write statement to clear ISFR here
}

// SW2 handler.  SW2 toggles the red LED
// in mode 0, the green LED in mode 1,
// and the blue LED in mode 2

void /* fill in appropriate handler name here */ {

	// Fill instruction to clear pending IRQ here

	// Check that it is SW2 pressesd
	if(// Fill appropriate statement here) {
		switch(mode) {
		case 0:
			GPIOE->PTOR |= (1 << REDLED);
			break;

		case 1:
			GPIOD->PTOR |= (1 << GREENLED);
			break;

		case 2:
			GPIOE->PTOR |= (1 << BLUELED);
			break;

		} // switch

        // Write statement to clear ISFR here
	}
}

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

    // Initialize LEDs
    initLEDs();

    // Switch off LEDs
    ledOff();

    // Initialize SW2 and SW3 interrupts
    initSW2Interrupt();
    initSW3Interrupt();

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

