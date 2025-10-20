/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271BusyWait.c
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

/*
 * @brief   Application entry point.
 */

#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

void initLEDs() {
	// Set up the clock gating
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK |
			SIM_SCGC5_PORTE_MASK);

	// Set up the pin PCR values
	PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_PIN] = PORT_PCR_MUX(1);
	GPIOD->PDDR |= (1 << GREEN_PIN);

	PORTE->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;

	PORTE->PCR[RED_PIN] = PORT_PCR_MUX(1);
	PORTE->PCR[BLUE_PIN] = PORT_PCR_MUX(1);
	GPIOE->PDDR |= ((1 << BLUE_PIN) | (1 << RED_PIN));
}

void toggleLED(TLED led) {
	switch(led) {
	case RED:
		GPIOE->PTOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PTOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOE->PTOR |= (1 << BLUE_PIN);
		break;
	}
}

void offLED(TLED led) {
	switch(led) {
	case RED:
		GPIOE->PSOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PSOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOE->PSOR |= (1 << BLUE_PIN);
		break;
	}
}
/*!
 * @brief Task responsible for printing of "Hello world." message.
 */

// Busy-wait delay for specific number of ms
// Not accurate but close enough.
#define MULT	3600000
void delayMS(int ms) {
	int clocks = (int)(ms / 1000.0 * (float) MULT);
	while(clocks) {
        __asm volatile ("nop");
        clocks--;
	}
}

/* We will print hello every 300 ms
 	 And:
 	 	 Toggle the RED LED every 600 ms
 	 	 Toggle the GREEN LED every 900 ms
 	 	 Toggle the BLUE LED every 1200 ms
 	 */

static void manageTasks() {
	int count=1;
	while(1) {
		switch(count) {
		case 1:
			PRINTF("Hello!\r\n");
			break;

		case 2:
			toggleLED(RED);
			break;

		case 3:
			toggleLED(GREEN);
			break;

		case 4:
			toggleLED(BLUE);
			break;
		}
		count++;
		if(count > 4)
			count=1;
		delayMS(300);
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

    initLEDs();
    offLED(RED);
    offLED(GREEN);
    offLED(BLUE);
    manageTasks();

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

