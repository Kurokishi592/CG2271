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

// Tilt Switch Configuration
// Tilt switch connected to PTC5
// Wiring: VCC to one end, other end to PTC5 with pull-down resistor
// When tilted ON: pin reads HIGH
// When tilted OFF: pin reads LOW (this triggers interrupt)

#define TILT_SWITCH     5
#define TILT_SWITCH_PORT PORTC
#define TILT_SWITCH_GPIO GPIOC
#define TILT_SWITCH_PORT_NAME "C"

// Initialize tilt switch interrupt
void initTiltSwitchInterrupt() {
  // Enable clock for PORTC
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

  // Disable interrupts before configuring
  NVIC_DisableIRQ(PORTC_PORTD_IRQn);

  // Set PTC5 to GPIO
  PORTC->PCR[TILT_SWITCH] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[TILT_SWITCH] |= PORT_PCR_MUX(1);

  // Enable pull-down resistor (PS bit controls pull direction, PE enables pull)
  // Note: If using pull-up instead, set PS_MASK
  PORTC->PCR[TILT_SWITCH] &= ~PORT_PCR_PS_MASK;  // Pull-down (PS=0)
  PORTC->PCR[TILT_SWITCH] |= PORT_PCR_PE_MASK;   // Enable pull

  // Set as input
  GPIOC->PDDR &= ~(1 << TILT_SWITCH);

  // Configure interrupt for falling edge (tilt switch OFF)
  // 0b1010 = falling edge detection
  PORTC->PCR[TILT_SWITCH] &= ~PORT_PCR_IRQC_MASK;
  PORTC->PCR[TILT_SWITCH] |= PORT_PCR_IRQC(0b1010);

  // Set NVIC priority
  NVIC_SetPriority(PORTC_PORTD_IRQn, 192);

  // Clear pending interrupts
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

  // Enable interrupts
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
  
  PRINTF("Tilt switch initialized on PTC%d\r\n", TILT_SWITCH);
}



int mode = 0;

// Tilt switch interrupt handler
// Triggered when tilt switch turns OFF (falling edge)
void PORTC_PORTD_IRQHandler() {
  // Clear pending IRQ for PORTC/PORTD
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

  // Check that tilt switch was triggered
  if(PORTC->ISFR & (1 << TILT_SWITCH)) {
    // Tilt switch turned OFF - do something here
    PRINTF("Tilt Switch OFF - Event triggered!\r\n");
    
    // Optional: Perform action (toggle LED, set flag, etc.)
    // Example: Set a flag to indicate tilt event
    mode = (mode + 1) % 2;  // Toggle between 0 and 1
    PRINTF("Mode: %d\r\n", mode);
    
    // Simple debounce delay (~20ms)
    volatile int i;
    for(i = 0; i < 160000; i++);
  }

  // Write a 1 to clear the ISFR bit
  PORTC->ISFR |= (1 << TILT_SWITCH);
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

  PRINTF("\r\n=========================\r\n");
  PRINTF("Tilt Switch Interrupt Demo\r\n");
  PRINTF("=========================\r\n");
  PRINTF("Tilt switch on PTC%d\r\n", TILT_SWITCH);
  PRINTF("Waiting for tilt switch OFF event...\r\n\r\n");

  // Initialize tilt switch interrupt
  initTiltSwitchInterrupt();

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
