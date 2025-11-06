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
#include "fsl_common.h"

//RTOS Header Files
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


SemaphoreHandle_t arm_state_signal; //Semaphore handling arm/disarmed
SemaphoreHandle_t alarm_signal; //Semaphore handling LDR sensor

/* Initialize interrupts for SW2 and SW3 */

// TILTSWITCH is connected to PTC5, SW3 to PTA4

#define INTERRUPTPIN 6 //PTD6
#define TILTSWITCH   5 //PTC5
#define SW3 4 //PTA4

void initLEDs() {
  SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);
  PORTD->PCR[INTERRUPTPIN] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[INTERRUPTPIN] |= PORT_PCR_MUX(1);
  // Set PDDR
  GPIOD->PDDR |= (1 << INTERRUPTPIN);
}

// Switch off all LEDs
void ledOff() {
  GPIOD->PCOR |= (1 << INTERRUPTPIN);
}

// Switch on all LEDs
void ledOn() {
  GPIOD->PSOR |= (1 << INTERRUPTPIN);
}

// SW3 is connected to PTA4
void initSW3Interrupt() {
  // Enable clock for PORTA
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Disable interrupts before configuring
  NVIC_DisableIRQ(PORTA_IRQn);

  // Set PTC5 to GPIO
  PORTA->PCR[SW3] &= ~PORT_PCR_MUX_MASK;
  PORTA->PCR[SW3] |= PORT_PCR_MUX(1);

  // Enable pull-up resistor (PS bit) and pull enable (PE bit)
  PORTA->PCR[SW3] &= ~PORT_PCR_PS_MASK;
  PORTA->PCR[SW3] |= PORT_PCR_PS(1);
  PORTA->PCR[SW3] &= ~PORT_PCR_PE_MASK;
  PORTA->PCR[SW3] |= PORT_PCR_PE(1);

  // Set as input
  GPIOA->PDDR &= ~(1 << SW3);

  // Configure interrupt for falling edge
  PORTA->PCR[SW3] &= ~PORT_PCR_IRQC_MASK;
  PORTA->PCR[SW3] |= PORT_PCR_IRQC(0b1001);

  // Set NVIC priority to the lowest (192)
  NVIC_SetPriority(PORTA_IRQn, 192); //TODO: upgrade this priority to override tilt

  // Clear pending interrupts
  NVIC_ClearPendingIRQ(PORTA_IRQn);

  // Enable interrupts
  NVIC_EnableIRQ(PORTA_IRQn);
}

// TILTSWITCH is connected to PTC3
void initTILTSWITCHInterrupt() {
  // Enable clock for PORTC
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

  // Disable interrupts before configuring
  NVIC_DisableIRQ(PORTC_PORTD_IRQn);

  // Set PTC5 to GPIO
  PORTC->PCR[TILTSWITCH] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[TILTSWITCH] |= PORT_PCR_MUX(1);

  // Enable pull-up resistor (PS bit) and pull enable (PE bit)
  PORTC->PCR[TILTSWITCH] &= ~PORT_PCR_PS_MASK;
  PORTC->PCR[TILTSWITCH] |= PORT_PCR_PS(1);
  PORTC->PCR[TILTSWITCH] &= ~PORT_PCR_PE_MASK;
  PORTC->PCR[TILTSWITCH] |= PORT_PCR_PE(1);

  // Set as input
  GPIOC->PDDR &= ~(1 << TILTSWITCH);

  // Configure interrupt for falling edge
  PORTC->PCR[TILTSWITCH] &= ~PORT_PCR_IRQC_MASK;
  PORTC->PCR[TILTSWITCH] |= PORT_PCR_IRQC(0b1001);

  // Set NVIC priority to the lowest (192)
  NVIC_SetPriority(PORTC_PORTD_IRQn, 192);

  // Clear pending interrupts
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

  // Enable interrupts
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

//Called when task is
//static void armBoardTask(void *p) {
//	//TODO: Refactor to make this a task
//	while(1) {
//		//Wait for armed state
//		if (xSemaphoreTake(arm_state_signal, portMAX_DELAY) == pdTRUE) {
//			//Do shit here
//			  GPIOD->PSOR = (1 << INTERRUPTPIN);  // Toggle mode
//		}
//	}
//}
//
//static void disarmBoardTask(void *p) {
//	//TODO:  Refactor to make this a task
//	while(1) {
//		//Wait for armed state
//		if (xSemaphoreTake(arm_state_signal, 0) == pdTRUE) {
//			//Do shit here
//			  GPIOD->PCOR = (1 << INTERRUPTPIN);  // Toggle mode
//		}
//	}
//}

// TILTSWITCH IRQHandler.  TILTSWITCH switches on the red LED
void PORTC_PORTD_IRQHandler() {
  // Clear pending IRQ for PORTC
	NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

	// Check that it is TILTSWITCH pressed
	if (PORTC->ISFR & 1 << TILTSWITCH) {
	  GPIOD->PSOR = (1 << INTERRUPTPIN);  // Toggle mode
	  BaseType_t hpw;
	  xSemaphoreGiveFromISR(arm_state_signal,&hpw);
	  portYIELD_FROM_ISR(hpw);

	}
	// Write a 1 to clear the ISFR bit
	PORTC->ISFR |= (1 << TILTSWITCH);
}


// SW3 IRQHandler.  SW3 switches on the red LED
void PORTA_IRQHandler() {
  // Clear pending IRQ for PORTA
  NVIC_ClearPendingIRQ(PORTA_IRQn);

  // Check that it is SW2 pressed
  if (PORTA->ISFR & 1 << SW3) {
	  GPIOD->PCOR = (1 << INTERRUPTPIN);  // Toggle mode
	  BaseType_t hpw;
	  xSemaphoreTakeFromISR(arm_state_signal, &hpw);
	  portYIELD_FROM_ISR(hpw);
      // Clear interrupt flag correctly
      PORTA->PCR[SW3] |= PORT_PCR_ISF_MASK;
  }
    // Write a 1 to clear the ISFR bit
    PORTA->ISFR |= (1 << SW3);
}

//BUZZER CODE SECTION

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
// TODO: Change this to utilise another clock (LDR using same timer for PWM is not good)

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
void playHappyTuneTask() {

	while(1) {

        if (xSemaphoreTake(arm_state_signal, portMAX_DELAY) == pdTRUE) {
            // We took it, so put it back to keep it "ON"
            xSemaphoreGive(arm_state_signal);
        }

		while (xSemaphoreTake(arm_state_signal, portMAX_DELAY) == pdTRUE) {
			xSemaphoreGive(arm_state_signal);
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
			vTaskDelay(500);
		}
	}
}
// --- END: Added function to play a simple, high-pitched tune ---

// --- START: Added function to play a sad/police siren wailing tone ---
void playPoliceSirenTask() {
//	PRINTF("Playing Police Siren...\r\n");
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

// ADC Section For LDR

// Define the baud rate for the debug console
#define DEBUG_CONSOLE_BAUD_RATE 9600

/* Pin Definitions */
#define LED_PIN     30 // PTE30 - TPM0_CH3
#define LDR_PIN     20 // PTE20 - ADC0_SE0
#define ADC_CHANNEL 0

// Define the maximum ADC reading value (12-bit conversion)
#define MAX_ADC_VALUE 4095

// Define the EMA smoothing factor (Alpha). Closer to 0 for more smoothing.
#define SMOOTHING_ALPHA 0.08f

typedef enum {
    LED
} TDEVICE;

volatile int adcValue = 0;
volatile int newDataReady = 0;
volatile float filteredAdcValue = 0.0f; // Stores the smoothed ADC value

//void setMCGIRClk() {
//    MCG->C1 &= ~MCG_C1_CLKS_MASK;
//    // Choose MCG clock source of 01 for LIRC and enable IRCLKEN
//    MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));
//
//    // Set IRCS to 1 to choose 8 MHz clock
//    MCG->C2 |= MCG_C2_IRCS_MASK;
//
//    // Choose FCRDIV of 0 for divisor of 1
//    MCG->SC &= ~MCG_SC_FCRDIV_MASK;
//    MCG->SC |= MCG_SC_FCRDIV(0b0);
//
//    // Choose LIRC_DIV2 of 0 for divisor of 1
//    MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
//    MCG->MC |= MCG_MC_LIRC_DIV2(0b0);
//}
//
///**
// * Set TPM Clock Source to MCGIRCLK (8 MHz)
// */
//void setTPMClock(){
//    setMCGIRClk();
//
//    // Choose MCGIRCLK (8 MHz) as TPM clock source
//    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
//    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
//}
//
///**
// * Initialize PWM for LED control on PTE30
// * LED connected to: PTE30 TPM0 CH3 ALT3
// * TODO: Change this to utilise another clock.
// */
//void initPWM() {
//    // Turn on clock gating to TPM0
//    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
//
//    // Turn on clock gating to Port E
//    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
//
//    // Set the pin multiplexor for PWM
//    PORTE->PCR[LED_PIN] &= ~PORT_PCR_MUX_MASK;
//    PORTE->PCR[LED_PIN] |= PORT_PCR_MUX(0b11);  // ALT3 for TPM0_CH3
//
//    // Set pin to output
//    GPIOE->PDDR |= (1 << LED_PIN);
//
//    // Set up TPM0
//    // Turn off TPM0 and clear the prescalar field
//    TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
//
//    // Set prescalar of 128
//    TPM0->SC |= TPM_SC_PS(0b111);
//
//    // Select centre-aligned PWM mode
//    TPM0->SC |= TPM_SC_CPWMS_MASK;
//
//    // Initialize count to 0
//    TPM0->CNT = 0;
//
//    // PWM frequency = 8 MHz / 128 = 62500 Hz
//    // For 500Hz PWM: MOD = 62500/500 = 125
//    TPM0->MOD = 125;
//
//    // Configure Channel 3 for LED
//    // MS=10, ELS=01 (Reverse PWM - LED is active low)
//    TPM0->CONTROLS[3].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK);
//    TPM0->CONTROLS[3].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSA(1));
//}
//
///**
// * Start PWM operation
// */
//void startPWM() {
//    TPM0->SC |= TPM_SC_CMOD(0b01);
//}
//
///**
// * Stop PWM operation
// */
//void stopPWM() {
//    TPM0->SC &= ~TPM_SC_CMOD_MASK;
//}

/**
 * Set PWM duty cycle (brightness)
 * @param device: Which device to control (LED)
 * @param percent: Brightness 0-100%
 */
void setPWM(int device, int percent) {
    // Limit to 0-100 range
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    int value = (int)((percent / 100.0) * (double) TPM0->MOD);

    switch(device){
        case LED:
            TPM0->CONTROLS[3].CnV = value;
            break;
        default:
            PRINTF("Invalid device.\r\n");
    }
}

/**
 * Initialize ADC for LDR reading on PTE20
 * LDR connected to: PTE20 ADC0_SE0
 */
void initADC() {
    // Configure interrupt
    NVIC_DisableIRQ(ADC0_IRQn);
    NVIC_ClearPendingIRQ(ADC0_IRQn);

    // Enable clock gating to ADC0
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    // Enable clock gating to Port E
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Set PTE20 (ADC0_SE0) to ADC mode
    PORTE->PCR[LDR_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LDR_PIN] |= PORT_PCR_MUX(0);  // MUX=0 for ADC/Analog

    // Configure the ADC
    // Enable ADC interrupt when conversion completes
    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

    // Select single-ended ADC (not differential)
    ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
    ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);

    // Set 12 bit conversion (0-4095 range)
    ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
    ADC0->CFG1 |= ADC_CFG1_MODE(0b01);  // 12-bit mode

    // Use software trigger (start conversion manually)
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;

    // Use VALTH and VALTL as reference voltage
    ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
    ADC0->SC2 |= ADC_SC2_REFSEL(0b01);

    // Don't use hardware averaging
    ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
    ADC0->SC3 |= ADC_SC3_AVGE(0);

    // Use continuous conversion mode
    ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
    ADC0->SC3 |= ADC_SC3_ADCO(1);  // Continuous conversion

    // Set interrupt priority (lowest)
    NVIC_SetPriority(ADC0_IRQn, 192);
    NVIC_EnableIRQ(ADC0_IRQn);

    PRINTF("ADC initialized for continuous conversion\r\n");
}

/**
 * Start ADC conversion on specified channel
 * @param channel: ADC channel number (0 for PTE20)
 */
void startADC(int channel) {
    PRINTF("Started ADC on channel %d\r\n", channel);

    // Select channel and start conversion
    ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
    ADC0->SC1[0] |= ADC_SC1_ADCH(channel);
}

/**
 * ADC Interrupt Handler
 * Called automatically when ADC conversion completes
 */
void ADC0_IRQHandler() {
    NVIC_ClearPendingIRQ(ADC0_IRQn);

    if(ADC0->SC1[0] & ADC_SC1_COCO_MASK){
        adcValue = ADC0->R[0]; // Read the new ADC value (0-4095)

        // Apply Exponential Moving Average (EMA) filter
        if (filteredAdcValue == 0.0f) {
            // Initial state: set filter value to the first reading
            filteredAdcValue = (float)adcValue;
        } else {
            // V_filtered = Alpha * V_new + (1 - Alpha) * V_filtered_old
            filteredAdcValue = (SMOOTHING_ALPHA * (float)adcValue) +
                               ((1.0f - SMOOTHING_ALPHA) * filteredAdcValue);
        }

        newDataReady = 1;      // Set flag to signal main loop

        // --- MODIFIED BRIGHTNESS CALCULATION (Using Smoothed Value) ---

        // Use the smoothed value for inversion. Note: We cast to int for inversion,
        // but keep the double/float for the division/percentage calculation.

        // Light Meter Function: Bright LDR light -> BRIGHT LED (low CnV value)
        int invertedAdc = MAX_ADC_VALUE - (int)filteredAdcValue;
        int brightness = (int)(((float)invertedAdc / (float)MAX_ADC_VALUE) * 100.0f);

        // Set LED brightness using the smoothed percentage
        setPWM(LED, brightness);
    }
}

//static void convertADCTask(*p) {
//	//TODO: Refactor to make ADC conversion into a task (from IRQ)
//}
//
//static void setPWMTask(*p) {
//	//TODO: Refactor to make setting PWM a task
//}

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

  // Initialize and switch off LEDs
  initLEDs();
  ledOff();

  // Initialize interrupts for SW3, TILTSWITCH and ADC
  initSW3Interrupt();
  initTILTSWITCHInterrupt();

  // Set the TPM clock
  setTPMClock();
  // initTimer() is NOT called as we don't need the TPM1 interrupt for fading.

  initPWM(); // Initializes TPM0 for buzzer PWM

//  initADC();
//  setPWM(LED, 0);  // Start with LED off
//
//  startADC(ADC_CHANNEL);  // Start continuous ADC conversion

  arm_state_signal = xSemaphoreCreateBinary();
  xTaskCreate(playHappyTuneTask, "task_happy", configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
//  xTaskCreate(playPoliceSirenTask, "task_alert", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);
  vTaskStartScheduler();

  /* Force the counter to be placed into memory. */
  volatile static int i = 0 ;
  /* Enter an infinite loop, just incrementing a counter. */
  while(1) {
    i++ ;
    /* 'Dummy' NOP to allow source level single stepping of
     * tight while() loop */
    __asm volatile ("nop");
  }
  return 0 ;
}
