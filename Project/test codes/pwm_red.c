#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#define LED_PIN     30
#define LDR_PIN     20
#define ADC_CHANNEL 0

typedef enum {
    LED
} TDEVICE;


volatile int adcValue = 0;
volatile int newDataReady = 0;

void setMCGIRClk() {
    MCG->C1 &= ~MCG_C1_CLKS_MASK;
    MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));
    
    MCG->C2 |= MCG_C2_IRCS_MASK;
    
    MCG->SC &= ~MCG_SC_FCRDIV_MASK;
    MCG->SC |= MCG_SC_FCRDIV(0b0);
    
    MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
    MCG->MC |= MCG_MC_LIRC_DIV2(0b0);
}

void setTPMClock(){
    setMCGIRClk();
    
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
}

void initPWM() {
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
    
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    
    PORTE->PCR[LED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LED_PIN] |= PORT_PCR_MUX(0b11);
    
    GPIOE->PDDR |= (1 << LED_PIN);
    
    TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
    
    TPM0->SC |= TPM_SC_PS(0b111);
    
    TPM0->SC |= TPM_SC_CPWMS_MASK;
    
    TPM0->CNT = 0;
    
    TPM0->MOD = 125;
    
    TPM0->CONTROLS[3].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSB_MASK);
    TPM0->CONTROLS[3].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSA(1));
}

void startPWM() {
    TPM0->SC |= TPM_SC_CMOD(0b01);
}

void stopPWM() {
    TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

void setPWM(int device, int percent) {
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

void initADC() {
    NVIC_DisableIRQ(ADC0_IRQn);
    NVIC_ClearPendingIRQ(ADC0_IRQn);
    
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    
    PORTE->PCR[LDR_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LDR_PIN] |= PORT_PCR_MUX(0);
    
    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;
    
    ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
    ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);
    
    ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
    ADC0->CFG1 |= ADC_CFG1_MODE(0b01);
    
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
    
    ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
    ADC0->SC2 |= ADC_SC2_REFSEL(0b00);
    
    ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
    ADC0->SC3 |= ADC_SC3_AVGE(0);
    
    ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
    ADC0->SC3 |= ADC_SC3_ADCO(1);
    
    NVIC_SetPriority(ADC0_IRQn, 192);
    NVIC_EnableIRQ(ADC0_IRQn);
    
    PRINTF("ADC initialized for continuous conversion\r\n");
}

void startADC(int channel) {
    PRINTF("Started ADC on channel %d\r\n", channel);
    
    ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
    ADC0->SC1[0] |= ADC_SC1_ADCH(channel);
}

void ADC0_IRQHandler() {
    NVIC_ClearPendingIRQ(ADC0_IRQn);
    
    if(ADC0->SC1[0] & ADC_SC1_COCO_MASK){
        adcValue = ADC0->R[0];
        newDataReady = 1;
    }
}

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    PRINTF("\r\n===================================\r\n");
    PRINTF("LDR-Controlled LED Brightness Demo\r\n");
    PRINTF("===================================\r\n");
    PRINTF("LDR on PTE20 (ADC0_SE0)\r\n");
    PRINTF("LED on PTE30 (TPM0_CH3)\r\n");
    PRINTF("Continuous ADC conversion mode\r\n\r\n");

    setTPMClock();
    initPWM();
    initADC();
    setPWM(LED, 0);
    startPWM();
    startADC(ADC_CHANNEL);

    int brightness = 0;
    static int printCounter = 0;

    while(1) {
        if (newDataReady) {
            newDataReady = 0; 
            
            brightness = (int)((adcValue / 4095.0) * 100);

            setPWM(LED, brightness);

            if (printCounter++ >= 10000) { 
                PRINTF("ADC: %4d | Brightness: %3d%%\r\n", adcValue, brightness);
                printCounter = 0;
            }
        }
    }
    
    return 0;
}