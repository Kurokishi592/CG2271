/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271Lab5.c
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
// Configure ADC_SE0 on PTE20 and ADC_SE4a on PTE21

#define ADC_SE0			0
#define ADC_SE0_PIN		20
#define ADC_SE4a		4
#define ADC_SE4_PIN		21

void initADC() {
	// Configure interrupt
	NVIC_DisableIRQ(ADC0_IRQn);
	NVIC_ClearPendingIRQ(ADC0_IRQn);

	// Enable clock gating to ADC0
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// Enable clock gating to PTE
	// This is done when we initialize the PWM
	// but we want to make our function self-contained
	// so we do it again
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// Set PTE20 and PTE21 (ADC0_SE0 and ADC_SE4a) to ADC
	PORTE->PCR[ADC_SE0_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[ADC_SE0_PIN] |= PORT_PCR_MUX(0);
	PORTE->PCR[ADC_SE4_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[ADC_SE4_PIN] |= PORT_PCR_MUX(0);

	// Configure the ADC
	// Enable ADC interrupt
	ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

	// Select single-ended ADC
	ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
	ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);

	// Set 12 bit conversion
	ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
	ADC0->CFG1 |= ADC_CFG1_MODE(0b01);

	// Use software trigger
	ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;

	// Use VALTH and VALTL
	ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
	ADC0->SC2 |= ADC_SC2_REFSEL(0b01);

	// Don't use averaging
	ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
	ADC0->SC3 |= ADC_SC3_AVGE(0);

	// Switch off continuous conversion.
	ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
	ADC0->SC3 |= ADC_SC3_ADCO(0);


	// Highest priority
	NVIC_SetPriority(ADC0_IRQn, 0);
	NVIC_EnableIRQ(ADC0_IRQn);

}

int result[2];

void startADC(int channel) {
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(channel);

}

void ADC0_IRQHandler(){
	static int turn=0;

	NVIC_ClearPendingIRQ(ADC0_IRQn);
	if(ADC0->SC1[0] & ADC_SC1_COCO_MASK) {
		// Read the result into result[turn]
		result[turn] = ADC0->R[0];/* Statement to read result */
		turn = 1 - turn;
		if(turn == 0) {
			// Call startADC to convert PTE20
			startADC(ADC_SE0);
		} else {
			//Call startADC to convert PTE21
			startADC(ADC_SE4a);
		}
	}
}

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

// Our LEDs are connected to:
// Red: PTE31 TPM0 CH 4 ALT3
// Green: PTD5 TPM0 CH 5 ALT4
// Blue: PTE29 TPM0 CH 2 ALT3
void initPWM() {

	// Turn on clock gating to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// Turn on clock gating to Port D and E
	// Set up the clock gating
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK |
			SIM_SCGC5_PORTE_MASK);


	// Set the pin multiplexors
	PORTE->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;

	// PWM
	PORTE->PCR[RED_PIN] |= PORT_PCR_MUX(0b11);
	PORTE->PCR[BLUE_PIN] |= PORT_PCR_MUX(0b11);

	// Note: No error PWM is MUX4 and not MUX3
	// for PTD5
	PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(0b100);

	// Set pins to output
	GPIOD->PDDR |= (1 << GREEN_PIN);
	GPIOE->PDDR |= (1 << BLUE_PIN);
	GPIOE->PDDR |= (1 << RED_PIN);

	// Set up TPM0
	// Turn off TPM0 and clear the prescalar field
	TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

	// Set prescalar of 128
	TPM0->SC |= TPM_SC_PS(0b111);

	// Select centre-aligned PWM mode
	TPM0->SC |= TPM_SC_CPWMS_MASK;

	// Initialize count to 0
	TPM0->CNT = 0;

	// We nominally choose a PWM frequency of 250 Hz
	// TPM frequency = 128/8000 = 0.016
	// For 250Hz, our mod value = 4 / (2 x 0.016)
	TPM0->MOD=125;

	// Configure channels 2, 4 and 5
	// MS=10, ELS=01.
	// Note that this configures a REVERSE PWM signal
	// I.e. it sets when counting up and clears when counting
	// down. This is because the LEDs are active low.

	TPM0->CONTROLS[2].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK);
	TPM0->CONTROLS[2].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSA(1));
	TPM0->CONTROLS[4].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK);
	TPM0->CONTROLS[4].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSA(1));
	TPM0->CONTROLS[5].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK);
	TPM0->CONTROLS[5].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSA(1));

}

void startPWM() {
	TPM0->SC |= TPM_SC_CMOD(0b01);
}

void stopPWM() {
	TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

void setPWM(int LED, int percent) {

	int value = (int)((percent / 100.0) * (double) TPM0->MOD);
//	PRINTF("value = %d\r\n", value);
	switch(LED){
		case RED:
			TPM0->CONTROLS[4].CnV = value;
			break;
		case GREEN:
			TPM0->CONTROLS[5].CnV = value;
			break;
		case BLUE:
			TPM0->CONTROLS[2].CnV = value;
			break;

		default:
			PRINTF("Invalid LED.\r\n");
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
    setTPMClock();
    initPWM();
    setPWM(RED, 0);
    setPWM(GREEN, 0);
    setPWM(BLUE, 0);
    startPWM();

    initADC();
    startADC(ADC_SE0);
    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;

        // Convert values
        int red = (int) (result[0] / 4096.0 * 255.0);
        int green = (int) (result[1] / 4096.0 * 255.0);
        int blue = (red + green) / 2;

        setPWM(RED, red);
        setPWM(GREEN, green);
        setPWM(BLUE, blue);
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

