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


void initGPIO() {

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
	GPIOE->PDDR |= (1 << BLUE_PIN);
	GPIOE->PDDR |= (1 << RED_PIN);


}

// Initialize interrupt to trigger on pin PTA4 (SW3).

void initInterrupt() {

	// Disable interrupts
	NVIC_DisableIRQ(PORTA_IRQn);

	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

	// Ensure that we do not trigger the NMI by setting
	// PTA4 to GPIO
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_MUX(1);

	// Set pullup resistor
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PS_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PS(1);
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PE_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PE(1);

	// Set as input
	GPIOA->PDDR &= ~(1 << SWITCH_PIN);

	// Configure the interrupt for rising edge
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_IRQC_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_IRQC(0b1001);

	// Set NVIC priority to 192,
	// lowest priority
	NVIC_SetPriority(PORTA_IRQn, 192);

	// Clear pending interrupts and enable interrupts
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}


void ledOn(TLED led) {
	switch(led) {
	case RED:
		GPIOE->PCOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PCOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOE->PCOR |= (1 << BLUE_PIN);
		break;
	}
}

void ledOff(TLED led) {
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

// Our counter must now be volatile and global.
volatile int count = 0;

// Interrupt handler for PORT A
void PORTA_IRQHandler() {

	NVIC_ClearPendingIRQ(PORTA_IRQn);

	// We don't actually have to test that it is
	// PTA4 since there is only one interrupt
	// configured, but this shows how to do it.
	if(PORTA->ISFR & (1 << SWITCH_PIN)) {
		count = (count + 1) % 6;
		PRINTF("Count = %d\r\n", count);
	}

	// Write a 1 to clear the ISFR bit (yes, 1, not 0)
	PORTA->ISFR |= (1 << SWITCH_PIN);
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

    PRINTF("Interrupt Demo\r\n");

    /* Force the counter to be placed into memory. */
    /* Enter an infinite loop, just incrementing a counter. */
    initGPIO();
    initInterrupt();

    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);

    while(1) {

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
    return 0 ;
}

