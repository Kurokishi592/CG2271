/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    ISRSema.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

// LED Pins
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29
#define SW_PIN		4	// PTA4

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

void initLEDs() {
	// Enable clock gating.
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

	PORTE->PCR[RED_PIN] &= ~(PORT_PCR_MUX_MASK);
	PORTE->PCR[RED_PIN] |= PORT_PCR_MUX(1);
	PORTE->PCR[BLUE_PIN] &= ~(PORT_PCR_MUX_MASK);
	PORTE->PCR[BLUE_PIN] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_PIN] &= ~(PORT_PCR_MUX_MASK);
	PORTD->PCR[GREEN_PIN] |= PORT_PCR_MUX(1);

	GPIOE->PDDR |= ((1 << RED_PIN) | (1 << BLUE_PIN));
	GPIOD->PDDR |= (1 << GREEN_PIN);
}

void offLED(TLED led) {
	switch(led){
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

void toggleLED(TLED led) {
	switch(led){
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

// Set up IRQ on PTA4 (SW3)
void initIRQ(){
	NVIC_DisableIRQ(PORTA_IRQn);
	// Clock gating for port A
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

	// Choose GPIO function
	PORTA->PCR[SW_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SW_PIN] = PORT_PCR_MUX(1);

	// Set as input
	GPIOA->PDDR &= ~(1 << SW_PIN);

	PORTA->PCR[SW_PIN] &= ~PORT_PCR_PS_MASK;
	PORTA->PCR[SW_PIN] |= PORT_PCR_PS(1);

	PORTA->PCR[SW_PIN] &= ~PORT_PCR_PE_MASK;
	PORTA->PCR[SW_PIN] |= PORT_PCR_PE(1);

	// Choose IRQ triggering style
	PORTA->PCR[SW_PIN] &= ~PORT_PCR_IRQC_MASK;

	// Choose rising edge.
	PORTA->PCR[SW_PIN] |= PORT_PCR_IRQC(0b1001);

	NVIC_SetPriority(PORTA_IRQn, 0);
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

int blink = 0;

void PORTA_IRQHandler() {
	static int count=0;

	count++;
	PRINTF("ISR triggered. count = %d\r\n", count);
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	if(PORTA->ISFR & (1 << SW_PIN)) {
		if((count % 5) == 0) {
			blink = 1;
		}
	}
	PORTA->ISFR |= (1 << SW_PIN);
}

// Prints Hello every 500 ms
static void helloTask(void *p) {
	while(1) {
		PRINTF("Hello!\r\n");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

static void blinkLEDTask(void *p) {
	PRINTF("BlinkLED Task Started\r\n");
	while(1) {
		if(blink) {
			// Toggle the RED LED
			toggleLED(RED);
			blink = 0;
		}
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
    initLEDs();

    offLED(RED);
    offLED(GREEN);
    offLED(BLUE);

    initIRQ();
#endif

    PRINTF("Semaphore from ISR Demo\r\n");

    // Set the blink flag to 0
    blink = 0;

    // Create the blinky task
    xTaskCreate(blinkLEDTask, "blink_led",
    		configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);

    // Create the hello task
    xTaskCreate(helloTask, "hello",
    		configMINIMAL_STACK_SIZE+100, NULL, 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

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

