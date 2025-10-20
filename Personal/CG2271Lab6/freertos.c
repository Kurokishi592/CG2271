/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    freertos_demo.c
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
#include "FreeRTOS.h"
#include "task.h"
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
static void hello_task(void *pvParameters)
{
    for (;;)
    {
        PRINTF("Hello world.\r\n");
        vTaskDelay(pdMS_TO_TICKS(300));

    }
}

#define RED_DELAY_MS	600
#define GREEN_DELAY_MS	900
#define BLUE_DELAY_MS	1200

static void blinkRedTask(void *p) {
	while(1){
		toggleLED(RED);
		vTaskDelay(pdMS_TO_TICKS(RED_DELAY_MS));
	}
}
static void blinkGreenTask(void *p) {
	while(1){
		toggleLED(GREEN);

		vTaskDelay(pdMS_TO_TICKS(GREEN_DELAY_MS));
	}
}
static void blinkBlueTask(void *p) {
	while(1){
		toggleLED(BLUE);

		vTaskDelay(pdMS_TO_TICKS(BLUE_DELAY_MS));
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
    if(xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL) != pdPASS)
    	PRINTF("Hello task creation failed.\r\n");
    else
    	PRINTF("Hello task success\r\n");
    xTaskCreate(blinkRedTask, "blink_red", configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
    xTaskCreate(blinkGreenTask, "blink_green", configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
    xTaskCreate(blinkBlueTask, "blink_blue", configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);

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

