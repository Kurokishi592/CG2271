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


#define INTERRUPTPIN 6 //PTD6

void initLEDs() {
  SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);
  PORTD->PCR[INTERRUPTPIN] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[INTERRUPTPIN] |= PORT_PCR_MUX(1);
  // Set PDDR
  GPIOD->PDDR |= (1 << INTERRUPTPIN);
}

// Switch off all LEDs
void ledOff() {
  GPIOD->PSOR |= (1 << INTERRUPTPIN);
}

// Switch on all LEDs
void ledOn() {
  GPIOD->PCOR |= (1 << INTERRUPTPIN);
}
/* Initialize interrupts for SW2 and SW3 */

// SW2 is connected to PTC5, SW3 to PTA4

#define SW2     5
//#define SW3     4

// SW2 is connected to PTC3
void initSW2Interrupt() {
  // Enable clock for PORTC
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

  // Disable interrupts before configuring
  NVIC_DisableIRQ(PORTC_PORTD_IRQn);

  // Set PTC5 to GPIO
  PORTC->PCR[SW2] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[SW2] |= PORT_PCR_MUX(1);

  // Enable pull-up resistor (PS bit) and pull enable (PE bit)
  PORTC->PCR[SW2] &= ~PORT_PCR_PS_MASK;
  PORTC->PCR[SW2] |= PORT_PCR_PS(1);
  PORTC->PCR[SW2] &= ~(PORT_PCR_PE_MASK);
  PORTC->PCR[SW2] |= PORT_PCR_PE(1);

  // Set as input
  GPIOC->PDDR &= ~(1 << SW2);

  // Configure interrupt for falling edge
  PORTC->PCR[SW2] &= ~PORT_PCR_IRQC_MASK;
  PORTC->PCR[SW2] |= PORT_PCR_IRQC(0b1001);

  // Set NVIC priority to the lowest (192)
  NVIC_SetPriority(PORTC_PORTD_IRQn, 192);

  // Clear pending interrupts
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

  // Enable interrupts
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}


int mode = 0;


// SW2 handler.  SW2 toggles the red LED
// in mode 0, the green LED in mode 1,
// and the blue LED in mode 2

void PORTC_PORTD_IRQHandler() {
  // Clear pending IRQ for PORTC
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

  // Check that it is SW2 pressed
  if (PORTC->PCR[SW2] & PORT_PCR_ISF_MASK) {

      // Toggle LED (example)
      GPIOD->PTOR = (1 << INTERRUPTPIN);

      // Clear interrupt flag correctly
      PORTC->PCR[SW2] |= PORT_PCR_ISF_MASK;
  }
    // Write a 1 to clear the ISFR bit
    PORTC->ISFR |= (1 << SW2);
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

  // Initialize LEDs
  initLEDs();

  // Switch off LEDs
  ledOff();

  // Initialize SW2 and SW3 interrupts
  initSW2Interrupt();
//  initSW3Interrupt();

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
