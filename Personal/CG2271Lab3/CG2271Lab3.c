/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    timer_demo.c
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

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

/*
 * @brief   Application entry point.
 */

// Configure the MCG Internal Reference Clock
void setMCGIRClk() {
	// Choose LIRC as the clock source and enable LIRC
	MCG->C1 &= ~MCG_C1_CLKS_MASK;
	MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

	// Choose the 2 MHz clock
	MCG->C2 &= ~MCG_C2_IRCS_MASK;

	// Set FRCDIV to dividing factor of 1
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;
	MCG->SC |= MCG_SC_FCRDIV(0b0);

	// Choose LIRC_DIV2 to dividing factor of 2
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
	MCG->MC |= MCG_MC_LIRC_DIV2(0b001);

}

void initTimer() {

	// Disable TPM0 interrupt
	NVIC_DisableIRQ(TPM0_IRQn);

	// Initialize the MCG Internal Reference Clock
	setMCGIRClk();

	// Turn on the clock gating
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// Set clock source
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;

	// Use MCGIRCLK
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);

	// Turn off TPM0 and clear the prescalar mask
	TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

	// Set the prescalar and the TOIE bit (prescalar = 16)
	TPM0->SC |= ((TPM_SC_TOIE_MASK) | TPM_SC_PS(0x3));

	// Initialize the count to 0 (clearTPMx_CNT b4 setting TPMx_MOD)
	TPM0->CNT = 0;

	// Initialize modulo to create an
	// interval of 500 ms.
	TPM0->MOD=62500;

	// Set priority to highest
	NVIC_SetPriority(TPM0_IRQn,0);

	// Enable TPM0 IRQ
	NVIC_EnableIRQ(TPM0_IRQn);

}

void startTimer() {
	// Use TPM counter clock to increment
	TPM0->SC |= TPM_SC_CMOD(0b1);
}

// Turn off the timer.
void stopTimer() {
	TPM0->SC |= TPM_SC_CMOD(0b0);
}

// Our counter must now be volatile and global.
volatile int count = 0;


void TPM0_IRQHandler(){
 // Clear pending IRQ
 NVIC_ClearPendingIRQ(TPM0_IRQn);
 if(TPM0->STATUS & TPM_STATUS_TOF_MASK) {
   count = (count + 1) % 6;

   // Reset CNT to 0
   TPM0->CNT = 0;
   // Clear TOF bit
   TPM0->STATUS |= TPM_STATUS_TOF_MASK;
  }
}

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


int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    initGPIO();
    initTimer();
    PRINTF("TIMER DEMO\r\n");
    startTimer();

    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);

    /* Enter an infinite loop, just incrementing a counter. */
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

