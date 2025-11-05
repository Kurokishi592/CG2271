/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    ldr_led_control.c
 * @brief   LDR controlled LED brightness using PWM and ADC
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

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

void setMCGIRClk() {
    MCG->C1 &= ~MCG_C1_CLKS_MASK;
    // Choose MCG clock source of 01 for LIRC and enable IRCLKEN
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

/**
 * Set TPM Clock Source to MCGIRCLK (8 MHz)
 */
void setTPMClock(){
    setMCGIRClk();

    // Choose MCGIRCLK (8 MHz) as TPM clock source
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
}

/**
 * Initialize PWM for LED control on PTE30
 * LED connected to: PTE30 TPM0 CH3 ALT3
 */
void initPWM() {
    // Turn on clock gating to TPM0
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    // Turn on clock gating to Port E
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Set the pin multiplexor for PWM
    PORTE->PCR[LED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_PIN] |= PORT_PCR_MUX(0b11);  // ALT3 for TPM0_CH3

    // Set pin to output
    GPIOE->PDDR |= (1 << LED_PIN);

    // Set up TPM0
    // Turn off TPM0 and clear the prescalar field
    TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

    // Set prescalar of 128
    TPM0->SC |= TPM_SC_PS(0b111);

    // Select centre-aligned PWM mode
    TPM0->SC |= TPM_SC_CPWMS_MASK;

    // Initialize count to 0
    TPM0->CNT = 0;

    // PWM frequency = 8 MHz / 128 = 62500 Hz
    // For 500Hz PWM: MOD = 62500/500 = 125
    TPM0->MOD = 125;

    // Configure Channel 3 for LED
    // MS=10, ELS=01 (Reverse PWM - LED is active low)
    TPM0->CONTROLS[3].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK);
    TPM0->CONTROLS[3].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSA(1));
}

/**
 * Start PWM operation
 */
void startPWM() {
    TPM0->SC |= TPM_SC_CMOD(0b01);
}

/**
 * Stop PWM operation
 */
void stopPWM() {
    TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

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

int main(void) {
    /* Init board hardware */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    PRINTF("\r\n===================================\r\n");
    PRINTF("LDR-Controlled LED Brightness Demo\r\n");
    PRINTF("    (With EMA Smoothing Factor: %.2f)\r\n", SMOOTHING_ALPHA);
    PRINTF("===================================\r\n");
    PRINTF("LDR on PTE20 (ADC0_SE0)\r\n");
    PRINTF("LED on PTE30 (TPM0_CH3)\r\n");
    PRINTF("Continuous ADC conversion mode\r\n\r\n");

    // Initialize peripherals
    setTPMClock();
    initPWM();
    initADC();
    setPWM(LED, 0);  // Start with LED off
    startPWM();
    startADC(ADC_CHANNEL);  // Start continuous ADC conversion

    // --- MODIFIED SECTION FOR PRINTING LDR READINGS ---
    int printCounter = 0;
    const int PRINT_RATE = 10; // Print approximately every 10 ADC conversions (faster than before)
    // ---------------------------------------------------

    /* Main loop */
    while(1) {
        // Check the flag set by the ADC interrupt handler
        if (newDataReady) {
            // Increment the counter to control the print rate
            if (printCounter++ >= PRINT_RATE) {
                // --- PRINTING USES SMOOTHED VALUE ---
                // We use the same inverted logic here for printing:
                int invertedAdc = MAX_ADC_VALUE - (int)filteredAdcValue;
                int brightness = (int)(((float)invertedAdc / (float)MAX_ADC_VALUE) * 100.0f);

                // Print all three values for comparison
                PRINTF("Raw LDR: %4d | Filtered ADC: %4.0f | LED Brightness: %3d%%\r\n",
                       adcValue, filteredAdcValue, brightness);

                printCounter = 0; // Reset counter
            }
            newDataReady = 0; // Clear the flag, regardless of whether we printed or not
        }
    }
    return 0;
}
