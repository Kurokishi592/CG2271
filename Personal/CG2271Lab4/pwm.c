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

	MCG->C1 &= ~MCG_C1_CLKS_MASK;
	// Choose MCG clock source of 01 for LIRC
	// and set IRCLKEN to 1 to enable LIRC
	MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

	// Set IRCS to 1 to choose 8 MHz clock
	MCG->C2 |= MCG_C2_IRCS_MASK;

	// Choose FCRDIV of 0 for divisor of 1
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;
	MCG->SC |= MCG_SC_FCRDIV(0b0);

	// Choose LIRC_DIV2 of 0 for divisor of 1
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
	MCG->MC |= MCG_MC_LIRC_DIV2(0b0);

}

void setTPMClock(){

	// Set MCGIRCLK
	setMCGIRClk();

	// Choose MCGIRCLK (8 MHz)
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);

}

void initTimer() {

	// Disable TPM1 interrupt
	NVIC_DisableIRQ(TPM1_IRQn);

	// Turn on the clock gating
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

	// Turn off TPM0 and clear the prescalar mask
	TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

	// Set a prescalar of 128, and the TOIE mask
	TPM1->SC |= ((TPM_SC_TOIE_MASK) | TPM_SC_PS(0x7));

	// Initialize the count to 0
	TPM1->CNT = 0;

	// Initialize modulo.
	// For a PS of 128, time is 128/8000 ms = 0.016 ms
	// For 10ms, MOD = 10/0.016 = 625 (approx)
	TPM1->MOD = 625;

	// Set priority to high
	NVIC_SetPriority(TPM1_IRQn, 0);

	NVIC_EnableIRQ(TPM1_IRQn);

}

void startTimer() {
	// Use TPM counter clock to increment.
	TPM1->SC |= TPM_SC_CMOD(0b1);
}

// Turn off the timer.
void stopTimer() {
	TPM1->SC &= ~TPM_SC_CMOD_MASK;
}

// Our LEDs are connected to:
// Red: PTE31 TPM0 CH 4 ALT3
// Green: PTD5 TPM0 CH 5 ALT4
// Blue: PTE29 TPM0 CH 2 ALT3
void initPWM() {

	// Turn on clock gating to TPM0

	// Turn on clock gating to the ports

	// Set the pin multiplexors for PWM

	// Set pins to output

	// Set up TPM0
	// Turn off TPM0 and clear the prescalar field

	// Set prescalar of 128

	// Select centre-aligned PWM mode

	// Initialize count to TPM0->CNT = 0;

	// We nominally choose a PWM frequency of 250 Hz
	// Calculate and set the appropriate MOD value.

	// Configure the MSB:MSA and ELSB:ELSA bits for
	// all the relevant channels

}

void startPWM() {
	// Fill in code to start the PWM
}

void stopPWM() {
	// Fill in code to stop the PWM
}

void setPWM(int LED, int percent) {

	int value = (int)((percent / 100.0) * (double) TPM0->MOD);
//	PRINTF("value = %d\r\n", value);
	switch(LED){
		case RED:
			// Statement to set the RED LED value
			break;
		case GREEN:
			// Statement to set the GREEN LED value
			break;
		case BLUE:
			// Statement to set the BLUE LED value
			break;

		default:
			PRINTF("Invalid LED.\r\n");
	}

}

volatile int redVal=0, greenVal=0, blueVal=0;

void TPM1_IRQHandler(){

	NVIC_ClearPendingIRQ(TPM1_IRQn);

	static int dir = 0;

	if(TPM1->STATUS & TPM_STATUS_TOF_MASK) {

			if((!dir && redVal < 100) || (dir && redVal>0)) {
				if(!dir)
					redVal++;
				else
					redVal--;

		    	setPWM(RED, redVal);

			} else {
				if((!dir && greenVal < 100) || (dir && greenVal > 0)) {
					if(!dir)
						greenVal++;
					else
						greenVal--;

					setPWM(GREEN, greenVal);

				} else {
					if((!dir && blueVal < 100) || (dir && blueVal > 0)) {
						if(!dir)
							blueVal++;
						else
							blueVal--;

				    	setPWM(BLUE, blueVal);

					} else {
						dir = 1 - dir;
					}
				}
			}

		//	PRINTF("redVal=%d, greenVal=%d, blueVal=%d\r\n",
		//			redVal, greenVal, blueVal);

			TPM1->CNT=0;
			TPM1->STATUS |= TPM_STATUS_TOF_MASK;
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

    // Set the TPM clock
    setTPMClock();
    initTimer();
    initPWM();
    setPWM(RED, 0);
    setPWM(GREEN, 0);
    setPWM(BLUE, 0);
    PRINTF("PWM DEMO\r\n");
    startPWM();
    startTimer();

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
    }
    return 0 ;
}

