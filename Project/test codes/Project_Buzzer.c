/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    timer_demo.c
 * @brief   Application entry point for Buzzer Tone Generation.
 */
#include <stdio.h>
#include <stdint.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

// Buzzer is connected to PTE29, which corresponds to TPM0_CH2 (originally Blue LED pin)
#define BUZZER_PIN	29	// PTE29 (TPM0_CH2)

// Base clock frequency for TPM0, assuming 8MHz LIRC clock and a prescaler of 128 (0x7)
// Input Clock Freq = 8,000,000 Hz / 128 = 62,500 Hz
#define TPM0_INPUT_CLOCK_HZ 62500U

// --- START: Added Note Frequencies for Tunes ---
// Defines for common musical notes (A4 = 440Hz standard)
// These frequencies allow us to play recognizable melodies.
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_LOW_SIREN_START 220
#define NOTE_HIGH_SIREN_END 330


/*
 * @brief   Application entry point.
 */

// Configure the MCG Internal Reference Clock
void setMCGIRClk() {
	//
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

// Simple blocking delay function for tone duration.
// NOTE: This assumes the core clock is relatively slow and the loop
// counter value is found by trial and error for a specific target platform.
void delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		// Adjust this value based on your specific processor speed to achieve accurate 1ms delay
		volatile uint32_t delay_count = 10000;
		while (delay_count--) {
			__asm("nop"); // No Operation
		}
	}
}

// Initialize PWM for the Buzzer on TPM0_CH2
void initPWM() {

	// Turn on clock gating to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// Turn on clock gating to the ports
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; // Only need PORTE for BUZZER_PIN

	// Set the pin multiplexor for PWM (PTE29 -> TPM0_CH2 -> ALT3)
	PORTE->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[BUZZER_PIN] |= PORT_PCR_MUX(3);  // ALT3 for TPM0_CH2 (BUZZER)

	// Set pins to output (though PWM takes over)
	GPIOE->PDDR |= (1 << BUZZER_PIN);

	// Set up TPM0
	// Turn off TPM0 and clear the prescalar field
	TPM0->SC = 0;

	// Set prescalar of 128 (0x7). Clock frequency is 8MHz/128 = 62.5kHz
	TPM0->SC |= TPM_SC_PS(0x7);

	// Select centre-aligned PWM mode
	TPM0->SC |= TPM_SC_CPWMS_MASK;

	// Initialize count to 0
	TPM0->CNT = 0;

	// Initial MOD value (set low to be updated by setBuzzerFrequency)
	TPM0->MOD = 0xFFFF;

	// Configure the MSB:MSA and ELSB:ELSA bits for TPM0_CH2 (Buzzer)
	// Sets to High-True Pulse PWM (active high output)
	TPM0->CONTROLS[2].CnSC = (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1));
}

void startPWM() {
	// Use TPM counter clock to increment (CMOD=0b01)
	TPM0->SC |= TPM_SC_CMOD(0b01);
}

void stopPWM() {
	// Turn off the TPM counter (CMOD=0b00)
	TPM0->SC &= ~(TPM_SC_CMOD_MASK);
}

// Function to set the buzzer to a specific frequency and 50% duty cycle
void setBuzzerFrequency(uint32_t frequency_hz) {

	if (frequency_hz == 0) {
		// Stop the PWM signal to turn the buzzer off by setting CnV to 0 (0% duty cycle)
		TPM0->CONTROLS[2].CnV = 0;
		return;
	}

	// Calculate MOD value for Centre-Aligned PWM:
	// MOD = (TPM_INPUT_CLOCK / (2 * Frequency))
	uint32_t mod_value = (TPM0_INPUT_CLOCK_HZ / (2 * frequency_hz));

	// Calculate CnV for a 50% duty cycle in Centre-Aligned mode
	// CnV = MOD / 2 (approx)
	uint32_t duty_cycle_value = mod_value / 2;

	// Set the new period (MOD)
	TPM0->MOD = mod_value;

	// Set the new duty cycle (CnV). 50% duty cycle is optimal for a buzzer.
	TPM0->CONTROLS[2].CnV = duty_cycle_value;
}

// Function to play a tone for a specific duration
void playTone(uint32_t frequency_hz, uint32_t duration_ms) {
	// 1. Set the PWM frequency
	setBuzzerFrequency(frequency_hz);

	// 2. Start the PWM if it's not already running
	startPWM();

	// 3. Wait for the specified duration
	delay_ms(duration_ms);

	// 4. Stop the tone by setting frequency to 0 (CnV=0)
	setBuzzerFrequency(0);
}

// --- START: Added function to play a simple, high-pitched tune ---
void playHappyTune() {
	PRINTF("Playing Happy Tune...\r\n");

	// Play 8th notes (125ms duration) from a simple major scale sequence
	playTone(NOTE_C4, 125); // C
	delay_ms(10);
	playTone(NOTE_D4, 125); // D
	delay_ms(10);
	playTone(NOTE_E4, 125); // E
	delay_ms(10);
	playTone(NOTE_C5, 250); // C (Higher) - Quarter note

	delay_ms(100); // Short rest

	playTone(NOTE_G4, 125); // G
	delay_ms(10);
	playTone(NOTE_E4, 125); // E
	delay_ms(10);
	playTone(NOTE_C4, 300); // C (Low) - Dotted Quarter note
}
// --- END: Added function to play a simple, high-pitched tune ---

// --- START: Added function to play a sad/police siren wailing tone ---
void playPoliceSiren() {
	PRINTF("Playing Police Siren...\r\n");

	// The siren effect is created by quickly ramping the frequency up and down.
	const int delay_time = 5; // milliseconds of delay for each step
	const int steps = 10; // number of steps from min to max frequency
	int current_freq;

	for (int i = 0; i < 4; i++) { // Repeat the wail cycle 4 times
		// Wail UP (Low to High)
		for (int j = 0; j <= steps; j++) {
			// Linear interpolation of frequency
			current_freq = NOTE_LOW_SIREN_START + j * (NOTE_HIGH_SIREN_END - NOTE_LOW_SIREN_START) / steps;
			setBuzzerFrequency(current_freq);
			startPWM(); // Ensure PWM is started for the smooth transition
			delay_ms(delay_time);
		}

		// Wail DOWN (High to Low)
		for (int j = steps; j >= 0; j--) {
			current_freq = NOTE_LOW_SIREN_START + j * (NOTE_HIGH_SIREN_END - NOTE_LOW_SIREN_START) / steps;
			setBuzzerFrequency(current_freq);
			startPWM();
			delay_ms(delay_time);
		}
	}

	// Stop the sound completely after the wail sequence finishes
	setBuzzerFrequency(0);
}
// --- END: Added function to play a sad/police siren wailing tone ---


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
    // initTimer() is NOT called as we don't need the TPM1 interrupt for fading.
    initPWM(); // Initializes TPM0 for buzzer PWM
    PRINTF("BUZZER TONE DEMO\r\n");
    playHappyTune();
    delay_ms(500);
    playPoliceSiren();
    delay_ms(1000);
    return 0 ;
}
