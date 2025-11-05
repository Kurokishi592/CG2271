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

// LEDs
// RED: PTE31
// GREEN: PTD5
// BLUE: PTE29

#define REDLED      31
#define GREENLED    5
#define BLUELED     29

void initLEDs() {
  SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);
  PORTE->PCR[REDLED] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[REDLED] |= PORT_PCR_MUX(1);

  PORTE->PCR[BLUELED] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[BLUELED] |= PORT_PCR_MUX(1);

  PORTD->PCR[GREENLED] &= PORT_PCR_MUX_MASK;
  PORTD->PCR[GREENLED] |= PORT_PCR_MUX(1);

  // Set PDDR
  GPIOE->PDDR |= ((1 << REDLED) | (1 << BLUELED));
  GPIOD->PDDR |= (1 << GREENLED);
}

// Switch off all LEDs
void ledOff() {
  GPIOE->PSOR |= ((1 << REDLED) | (1 << BLUELED));
  GPIOD->PSOR |= (1 << GREENLED);
}

// Switch on all LEDs
void ledOn() {
  GPIOE->PCOR |= ((1 << REDLED) | (1 << BLUELED));
  GPIOD->PCOR |= (1 << GREENLED);
}
/* Initialize interrupts for SW2 and SW3 */

// SW2 is connected to PTC3, SW3 to PTA4

#define SW2     3
#define SW3     4

// SW2 is connected to PTC3
void initSW2Interrupt() {
  // Enable clock for PORTC
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

  // Disable interrupts before configuring
  NVIC_DisableIRQ(PORTC_PORTD_IRQn);

  // Set PTC3 to GPIO
  PORTC->PCR[SW2] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[SW2] |= PORT_PCR_MUX(1);

  // Enable pull-up resistor (PS bit) and pull enable (PE bit)
  PORTC->PCR[SW2] |= PORT_PCR_PS_MASK;
  PORTC->PCR[SW2] |= PORT_PCR_PE_MASK;

  // Set as input
  GPIOC->PDDR &= ~(1 << SW2);

  // Configure interrupt for falling edge
  PORTC->PCR[SW2] &= ~PORT_PCR_IRQC_MASK;
  PORTC->PCR[SW2] |= PORT_PCR_IRQC(0b1010);

  // Set NVIC priority to the lowest (192)
  NVIC_SetPriority(PORTC_PORTD_IRQn, 192);

  // Clear pending interrupts
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

  // Enable interrupts
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

// SW3 is connected to pin PTA4
void initSW3Interrupt() {
  // Enable clock for PORTA
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Disable interrupts before configuring
  NVIC_DisableIRQ(PORTA_IRQn);

  // Set PTA4 to GPIO
  PORTA->PCR[SW3] &= ~PORT_PCR_MUX_MASK;
  PORTA->PCR[SW3] |= PORT_PCR_MUX(1);

  // Enable pull-up resistor (PS bit) and pull enable (PE bit)
  PORTA->PCR[SW3] |= PORT_PCR_PS_MASK;
  PORTA->PCR[SW3] |= PORT_PCR_PE_MASK;

  // Set as input
  GPIOA->PDDR &= ~(1 << SW3);

  // Configure interrupt for falling edge
  PORTA->PCR[SW3] &= ~PORT_PCR_IRQC_MASK;
  PORTA->PCR[SW3] |= PORT_PCR_IRQC(0b1010);

  // Set NVIC priority to the lowest (192)
  NVIC_SetPriority(PORTA_IRQn, 192);

  // Clear pending interrupts
  NVIC_ClearPendingIRQ(PORTA_IRQn);

  // Enable interrupts
  NVIC_EnableIRQ(PORTA_IRQn);
}



int mode = 0;

// SW3 handler. SW3 toggles the mode
void PORTA_IRQHandler() {
  // Clear pending IRQ for PORTA
  NVIC_ClearPendingIRQ(PORTA_IRQn);

  // Check that SW3 was triggered
  if(PORTA->ISFR & (1 << SW3)) {
    // mode cycles from 0 to 1 to 2 to 0, etc.
    mode = (mode + 1) % 3;
  }

  // Write a 1 to clear the ISFR bit
  PORTA->ISFR |= (1 << SW3);
}

// SW2 handler.  SW2 toggles the red LED
// in mode 0, the green LED in mode 1,
// and the blue LED in mode 2

void PORTC_IRQHandler() {
  // Clear pending IRQ for PORTC
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);

  // Check that it is SW2 pressed
  if(PORTC->ISFR & (1 << SW2)) {
    switch(mode) {
    case 0:
      GPIOE->PTOR |= (1 << REDLED);
      break;

    case 1:
      GPIOD->PTOR |= (1 << GREENLED);
      break;

    case 2:
      GPIOE->PTOR |= (1 << BLUELED);
      break;

    } // switch

    // Write a 1 to clear the ISFR bit
    PORTC->ISFR |= (1 << SW2);
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

  // Initialize LEDs
  initLEDs();

  // Switch off LEDs
  ledOff();

  // Initialize SW2 and SW3 interrupts
  initSW2Interrupt();
  initSW3Interrupt();

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
