/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    MCXC444_Project.c
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

int sum=0;

#define COUNT		100000
#define NUM_TASKS	5

static void addOneTask(void *p) {
	for(int i=0; i<COUNT; i++) {
		sum++;
	}
	vTaskSuspend(NULL);
}

static void printTask(void *p) {
	while(1) {
		// Delay before printing
		PRINTF("Waiting ten seconds before printing results\r\n");
		vTaskDelay(pdMS_TO_TICKS(10000));
		PRINTF("Expected result: %d Actual: %d\r\n",
				NUM_TASKS * COUNT, sum);
		 vTaskSuspend(NULL);
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

    PRINTF("Race Demo\r\n");

    char taskName[12];

    for(int i=0; i<NUM_TASKS; i++) {
    	sprintf(taskName, "addone_%d", i);
    	xTaskCreate(addOneTask, taskName, configMINIMAL_STACK_SIZE+100, NULL, 2, NULL);
    }
    xTaskCreate(printTask, "print_task", configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);
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

