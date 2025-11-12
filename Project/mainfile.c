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
#include <string.h>
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
#include "queue.h"

QueueHandle_t dataQueue; // queue for ADC

SemaphoreHandle_t alarm_signal;  // Semaphore: request Alarm ON
SemaphoreHandle_t disarm_signal; // Semaphore: request Alarm OFF

static volatile BaseType_t stopFlag = 0;   // shared state

static TaskHandle_t taskHappyHandle = NULL;
static TaskHandle_t taskAlarmHandle = NULL;


// SemaphoreHandle_t adcValueMutex; // Not needed with queue
SemaphoreHandle_t systemStateMutex;
// Global structure to hold the current system state for UART transmission
typedef struct {
  uint8_t isArmed;    // Interpreted as Alarm Enabled (from ESP32 ALARM_ON/OFF)
  uint8_t isAlarming; // 1 when siren is active (e.g., LDR over threshold or IMU_TILT=1 while enabled)
} SystemState_t;

volatile SystemState_t currentState = {1, 0}; // Default: Alarm OFF (isArmed=1), Not Alarming

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

// ADC Section For LDR

// Define the baud rate for the debug console
#define DEBUG_CONSOLE_BAUD_RATE 9600

/* Pin Definitions */
#define LED_PIN     21 // PTE21 - TPM1_CH1
#define LDR_PIN     20 // PTE20 - ADC0_SE0
#define ADC_CHANNEL 0

// Define the maximum ADC reading value (12-bit conversion)
#define MAX_ADC_VALUE 4095

// Define the EMA smoothing factor (Alpha). Closer to 0 for more smoothing.
#define SMOOTHING_ALPHA 0.08f

typedef enum {
    LED
} TDEVICE;

volatile double adcValue = 0;
volatile int newDataReady = 0;
volatile float filteredAdcValue = 0.0f; // Stores the smoothed ADC value
volatile uint16_t ADC_THRESHOLD = 50; //default threshold

#define BAUD_RATE 9600
#define UART_TX_PTE22 	22
#define UART_RX_PTE23 	23
#define UART2_INT_PRIO	128

void initUART2(uint32_t baud_rate)
{
	NVIC_DisableIRQ(UART2_FLEXIO_IRQn);
	//enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	//Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	//connect UART pins for PTE22, PTE23
	PORTE->PCR[UART_TX_PTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PTE22] |= PORT_PCR_MUX(4);

	PORTE->PCR[UART_RX_PTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PTE23] |= PORT_PCR_MUX(4);

	// Set the baud rate
	uint32_t bus_clk = CLOCK_GetBusClkFreq();

	// This version of sbr does integer rounding.
	uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

	// Set SBR. Bits 8 to 12 in BDH, 0-7 in BDL.
	// MUST SET BDH FIRST!
	UART2->BDH &= ~UART_BDH_SBR_MASK;
	UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
	UART2->BDL = (uint8_t) (sbr &0xFF);

	// Disable loop mode
	UART2->C1 &= ~UART_C1_LOOPS_MASK;
	UART2->C1 &= ~UART_C1_RSRC_MASK;

	// Disable parity
	UART2->C1 &= ~UART_C1_PE_MASK;

	// 8-bit mode
	UART2->C1 &= ~UART_C1_M_MASK;

	//Enable RX interrupt
	UART2->C2 |= UART_C2_RIE_MASK;

	// Enable the receiver
	UART2->C2 |= UART_C2_RE_MASK;

	NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
	NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

#define MAX_MSG_LEN		256
char send_buffer[MAX_MSG_LEN];

#define QLEN	5
QueueHandle_t queue;
typedef struct tm {
	char message[MAX_MSG_LEN];
} TMessage;

void UART2_FLEXIO_IRQHandler(void)
{
	// Send and receive pointers
	static int recv_ptr=0, send_ptr=0;
	char rx_data;
  static char recv_buffer[MAX_MSG_LEN];

//VIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
	if(UART2->S1 & UART_S1_TDRE_MASK) // Send data
	{
		if(send_buffer[send_ptr] == '\0') {
			send_ptr = 0;

			// Disable the transmit interrupt
			UART2->C2 &= ~UART_C2_TIE_MASK;

			// Disable the transmitter
			UART2->C2 &= ~UART_C2_TE_MASK;
		}
		else {
			UART2->D = send_buffer[send_ptr++];
		}
	}

	if(UART2->S1 & UART_S1_RDRF_MASK)
	{
		TMessage msg;
		rx_data = UART2->D;
		recv_buffer[recv_ptr++] = rx_data;
		if(rx_data == '\n') {
			// Copy over the string
			BaseType_t hpw;
			recv_buffer[recv_ptr]='\0';
			strncpy(msg.message, recv_buffer, MAX_MSG_LEN);
			xQueueSendFromISR(queue, (void *)&msg, &hpw);
			portYIELD_FROM_ISR(hpw);
			recv_ptr = 0;
		}
	}
}

void sendMessage(char *message) {
	strncpy(send_buffer, message, MAX_MSG_LEN);

	// Enable the TIE interrupt
	UART2->C2 |= UART_C2_TIE_MASK;

	// Enable the transmitter
	UART2->C2 |= UART_C2_TE_MASK;
}

// --- UART RX task: parse ESP32 commands and raise semaphores accordingly ---
static void recvTask(void *p) {
  while(1) {
    TMessage msg;
    if(xQueueReceive(queue, (TMessage *) &msg, portMAX_DELAY) == pdTRUE) {
      // Trim CRLF
      char *line = msg.message;
      size_t n = strlen(line);
      while (n && (line[n-1] == '\r' || line[n-1] == '\n')) { line[--n] = '\0'; }

      if (strncmp(line, "ALARM_ON", 8) == 0) {
        // Turn alarm ON
        xSemaphoreGive(alarm_signal);
      } else if (strncmp(line, "ALARM_OFF", 9) == 0) {
        // Turn alarm OFF
        xSemaphoreGive(disarm_signal);
      } else if (strncmp(line, "TH_LIGHT=", 9) == 0) {
        // Set LDR threshold
        const char *pnum = line + 9; unsigned val = 0;
        while (*pnum >= '0' && *pnum <= '9') { val = val*10 + (unsigned)(*pnum - '0'); pnum++; }
        ADC_THRESHOLD = (uint16_t)val;
      } else if (strncmp(line, "IMU_TILT=", 9) == 0) {
        int tilt = (line[9] == '1') ? 1 : 0;
        if (tilt) {
          xSemaphoreGive(alarm_signal); // IMU tilt triggers alarm ON
        }
      }
    }
  }
}

// --- UART TX task: periodically report STATUS to ESP32 ---
static void sendTask(void *p) {
  char buffer[MAX_MSG_LEN];
  while(1) {
    uint8_t alarm_on = 0;
    uint16_t light_val = adcValue;

    // Map STATUS alarm to current alarm state
    taskENTER_CRITICAL();
    alarm_on = (currentState.isAlarming ? 1 : 0);
    taskEXIT_CRITICAL();

    snprintf(buffer, sizeof(buffer), "STATUS alarm=%u light=%u\n", (unsigned)alarm_on, (unsigned)light_val);
    sendMessage(buffer);
    vTaskDelay(pdMS_TO_TICKS(250)); // ~4 Hz
  }
}

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

// TILTSWITCH falling edge: if alarm currently ON -> request OFF, else request ON
void PORTC_PORTD_IRQHandler() {
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
  if (PORTC->ISFR & (1 << TILTSWITCH)) {
    BaseType_t hpw = pdFALSE;
    // Read current alarming state atomically
    uint8_t alarming;
    taskENTER_CRITICAL(); alarming = currentState.isAlarming; taskEXIT_CRITICAL();
    if (alarming) {
        xSemaphoreGiveFromISR(disarm_signal, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
  }
  PORTC->ISFR |= (1 << TILTSWITCH);
}


// State updater task
void stateControllerTask(void *p) {
  while (1) {
    // Only wait on the semaphore that makes sense for the current state,
    // with a short timeout to remain responsive to other events.
    uint8_t alarming;
    taskENTER_CRITICAL();
    alarming = currentState.isAlarming;
    taskEXIT_CRITICAL();

    if (!alarming) {
      // Alarm is currently OFF: watch for any ON request
      if (xSemaphoreTake(alarm_signal, pdMS_TO_TICKS(5)) == pdTRUE) {
        taskENTER_CRITICAL();
        currentState.isAlarming = 1;
        currentState.isArmed = 0;
        stopFlag = 1; // interrupt any ongoing tune immediately
        taskEXIT_CRITICAL();

        // Nudge both tone tasks so they can switch promptly
        xTaskNotify(taskHappyHandle, 0, eNoAction);
        xTaskNotify(taskAlarmHandle, 0, eNoAction);
      }
    } else {
      // Alarm is currently ON: watch for any OFF request
      if (xSemaphoreTake(disarm_signal, pdMS_TO_TICKS(5)) == pdTRUE) {
        taskENTER_CRITICAL();
        currentState.isAlarming = 0;
        currentState.isArmed = 1;
        stopFlag = 1; // interrupt siren immediately
        taskEXIT_CRITICAL();

        xTaskNotify(taskHappyHandle, 0, eNoAction);
        xTaskNotify(taskAlarmHandle, 0, eNoAction);
      }
    }

    // Cooperative yield to ensure quick preemption even under load
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

//BUZZER CODE SECTION




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
		}
	}
}

// Initialize PWM for the Buzzer on TPM0_CH2
// TODO: Change this to utilise another clock (LDR using same timer for PWM is not good)

void initTPM0PWM() {

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


////Plays a tone that as an interruptible task.
void playToneInterruptible(uint32_t freq, uint32_t duration_ms)
{
    uint32_t elapsed = 0;
	// 1. Set the PWM frequency
	setBuzzerFrequency(freq);

	// 2. Start the PWM if it's not already running
	startPWM();

    while (elapsed < duration_ms)
    {
        // Check if state was turned off
        if (stopFlag)
        {
        	setBuzzerFrequency(0);
            return; // Immediate exit
        }
		delay_ms(1);
		elapsed++;
    }

    setBuzzerFrequency(0);   // immediate cutoff after done playing
    vTaskDelay(pdMS_TO_TICKS(5)); //delay task to allow this to play later
    return;
}
// --- START: Added function to play a simple, high-pitched tune ---
void playHappyTuneTask(void *p) {
	SystemState_t currState;
	while(1) {
		currState = currentState;
		if (currState.isArmed == 1) {
			// Play 8th notes (125ms duration) from a simple major scale sequence
			stopFlag = 0;
			playToneInterruptible(NOTE_C4, 50); // C
			playToneInterruptible(0,1);
			playToneInterruptible(NOTE_D4, 50); // D
			playToneInterruptible(0,1);
			playToneInterruptible(NOTE_E4, 50); // E
			playToneInterruptible(0,1);
			playToneInterruptible(NOTE_C5, 50); // C (Higher) - Quarter note

			playToneInterruptible(0,1);

			playToneInterruptible(NOTE_G4, 50); // G
			playToneInterruptible(0,1);
			playToneInterruptible(NOTE_E4, 50); // E
			playToneInterruptible(0,1);
			playToneInterruptible(NOTE_C4, 50); // C (Low) - Dotted Quarter note
			playToneInterruptible(0,1);

            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
		} else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
	SystemState_t currState;

	while (1) {
		currState = currentState;
		if (currState.isAlarming == 1) {
			stopFlag = 0;
//			for (int i = 0; i < 2; i++) { // Repeat the wail cycle 4 times
				// Wail UP (Low to High)
				for (int j = 0; j <= steps; j++) {
					// Linear interpolation of frequency
					current_freq = NOTE_LOW_SIREN_START + j * (NOTE_HIGH_SIREN_END - NOTE_LOW_SIREN_START) / steps;
					playToneInterruptible(current_freq, delay_time);
				}

				// Wail DOWN (High to Low)
				for (int j = steps; j >= 0; j--) {
					current_freq = NOTE_LOW_SIREN_START + j * (NOTE_HIGH_SIREN_END - NOTE_LOW_SIREN_START) / steps;
					playToneInterruptible(current_freq, delay_time);
				}
//			}

            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
		} else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		}
	}
}
// --- END: Added function to play a sad/police siren wailing tone ---

///**
// * Initialize PWM for LED control on PTE30
// * LED connected to: PTE21 TPM1 CH0 ALT3
// * TODO: Change this to utilise another clock.
// */
void initTPM1PWM() {
    // Turn on clock gating to TPM0
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

    // Turn on clock gating to Port E
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Set the pin multiplexor for PWM
    PORTE->PCR[LED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_PIN] |= PORT_PCR_MUX(0b11);  // ALT3 for TPM1_CH1

    // Set pin to output
    GPIOE->PDDR |= (1 << LED_PIN);

    // Set up TPM1
    // Turn off TPM1 and clear the prescalar field
    TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

    // Set prescalar of 128
    TPM1->SC |= TPM_SC_PS(0b111);

    // Select centre-aligned PWM mode
    TPM1->SC |= TPM_SC_CPWMS_MASK;

    // Initialize count to 0
    TPM1->CNT = 0;

    // PWM frequency = 8 MHz / 128 = 62500 Hz
    // For 500Hz PWM: MOD = 62500/500 = 125
    TPM1->MOD = 125;

    // Configure Channel 1 for LED
    // MS=10, ELS=01 (Reverse PWM - LED is active low)
    TPM1->CONTROLS[1].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK);
    TPM1->CONTROLS[1].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSA(1));
}

///**
// * Start PWM operation
// */
void startLEDPWM() {
    TPM1->SC |= TPM_SC_CMOD(0b01);
}

///**
// * Stop PWM operation
// */
void stopLEDPWM() {
    TPM1->SC &= ~TPM_SC_CMOD_MASK;
}

/**
 * Set PWM duty cycle (brightness)
 * @param device: Which device to control (LED)
 * @param percent: Brightness 0-100%
 */
void setPWM(int device, double percent) {
    // Limit to 0-100 range
    int value = (int)((percent / 100) * (double) TPM1->MOD);
    switch(device){
        case LED:
            TPM1->CONTROLS[1].CnV = value;
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

    // Do NOT use continuous conversion mode; we poll this
    ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
    ADC0->SC1[0] &= ~ADC_SC1_AIEN_MASK;

}
/*
 * Read the ADC values manually from the channel
 */
uint16_t readADC(int channel) {
    ADC0->SC1[0] = channel & ADC_SC1_ADCH_MASK;   // start conversion

    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) { } // wait until you get the value

    return ADC0->R[0];
}

static void convertADCTask(void *p) {
    while (1) {
        uint16_t newValue = readADC(ADC_CHANNEL);
    	double brightness = (newValue - 3700) / (double) 300;
    	int converted = (int) ((1 - brightness) * (100));
        if (converted < 0) converted = 0;
        if (converted > 100) converted = 100;
        adcValue = converted;
//        PRINTF("adcValue: %d\r\n", converted);
        xQueueSend(dataQueue, &adcValue, 0);

	  // Trigger alarm on LOW light: adcValue below threshold
	  if (adcValue < ADC_THRESHOLD) {
		  xSemaphoreGive(alarm_signal);
	  }

        vTaskDelay(pdMS_TO_TICKS(10));  // sample every 10 ms (100 Hz)
    }
}

static void setPWMTask(void *p) {
	while (1){
        if (xQueueReceive(dataQueue, &adcValue, portMAX_DELAY)) {
    		//TODO: Add adc smoothening here

    		setPWM(LED, adcValue);
        }

		vTaskDelay(pdMS_TO_TICKS(10)); // update at same rate
	}
}
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


  // Initialize and switch off LEDs
  initLEDs();
  ledOff();

  // Initialize interrupts for SW3, TILTSWITCH and ADC
  initTILTSWITCHInterrupt();

  // Set the TPM clock
  setTPMClock();

  initTPM0PWM(); // Initializes TPM0 for buzzer PWM
  initTPM1PWM(); //Initialilzes TPM1 for LED PWM

  initADC();
  setPWM(LED, 0);  // Start with LED off
  startLEDPWM();  //Start LED PWM channel

  alarm_signal = xSemaphoreCreateBinary();  // Binary semaphore: request Alarm ON
  disarm_signal = xSemaphoreCreateBinary(); // Binary semaphore: request Alarm OFF
  dataQueue = xQueueCreate(10, sizeof(uint16_t)); //Create queue for ADC

  // Create the new mutex for the UART global state
	systemStateMutex = xSemaphoreCreateMutex();
	queue = xQueueCreate(QLEN, sizeof(TMessage)); // For UART receive task

  // Initialize UART2 after RX queue exists
  initUART2(BAUD_RATE);


//  adcValueMutex = xSemaphoreCreateMutex();	//Create mutex for ADC conversion
  xTaskCreate(stateControllerTask, "controller", configMINIMAL_STACK_SIZE+100, NULL, 4,NULL);
  // UART tasks for protocol with ESP32
  xTaskCreate(recvTask, "uart_recv", configMINIMAL_STACK_SIZE+120, NULL, 3, NULL);
  xTaskCreate(sendTask, "uart_send", configMINIMAL_STACK_SIZE+120, NULL, 3, NULL);
  xTaskCreate(playHappyTuneTask, "task_happy", configMINIMAL_STACK_SIZE+100, NULL, 2, &taskHappyHandle);
  xTaskCreate(playPoliceSirenTask, "task_alert", configMINIMAL_STACK_SIZE+100, NULL, 2, &taskAlarmHandle);
  xTaskCreate(convertADCTask, "task_adc", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);
  xTaskCreate(setPWMTask, "task_ledpwm", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);

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
