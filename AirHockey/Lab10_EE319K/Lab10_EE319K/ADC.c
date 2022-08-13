// ADC.c
// Runs on TM4C123
// Provide functions that initialize ADC0
// Last Modified: 11/15/2021 
// Student names: change this to your names or look very silly
// Last modification date: change this to the last modification date or look very silly

#include <stdint.h>
#include "../../inc/tm4c123gh6pm.h"

// ADC initialization function 
// Input: none
// Output: none
// measures from PD2, analog channel 5
void ADC_Init(void){ 
//	volatile unsigned long delay;
//	SYSCTL_RCGCGPIO_R |= 0x08;
//	while((SYSCTL_PRGPIO_R&0x08)==0){};
//	GPIO_PORTD_DIR_R &= ~0x04;
//  GPIO_PORTD_AFSEL_R |= 0x04;
//	GPIO_PORTD_DEN_R &= ~0x04;
//	GPIO_PORTD_AMSEL_R |= 0x04;
//		
//	SYSCTL_RCGCADC_R |= 0x0021;//adc5 0x0021
//	delay = SYSCTL_RCGCADC_R;
//	delay = SYSCTL_RCGCADC_R;
//	delay = SYSCTL_RCGCADC_R;
//	delay = SYSCTL_RCGCADC_R;
//	
//	ADC0_PC_R = 0x01;  					// 7) configure for 125K 
//  ADC0_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
//  ADC0_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
//  ADC0_EMUX_R &= ~0xF000;         // 10) seq3 is software trigger
//  ADC0_SSMUX3_R = (ADC0_SSMUX3_R&0xFFFFFFF0)+5; // 11) channel Ain5 (PD2)
//  ADC0_SSCTL3_R = 0x0006; 				// 12) no TS0 D0, yes IE0 END0
//	ADC0_IM_R &= ~0x0008; 					//DISABLE interrupt
//  ADC0_ACTSS_R |= 0x0008;         // 13) enable sample sequencer 3 
//	//ADC0_SAC_R = 20;
  volatile uint32_t delay;                         
//  SYSCTL_RCGC0_R |= 0x00010000; // 1) activate ADC0 (legacy code)
  SYSCTL_RCGCADC_R |= 0x00000001; // 1) activate ADC0
  SYSCTL_RCGCGPIO_R |= 0x18; // 1) activate clock for Port E and Port D
	
	
	delay = SYSCTL_RCGCGPIO_R;
	delay = SYSCTL_RCGCGPIO_R;
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
	
  GPIO_PORTE_DIR_R &= ~0x10;      // 3) make PE4 PE5 input
  GPIO_PORTE_AFSEL_R |= 0x10;     // 4) enable alternate function on PE4 PE5
  GPIO_PORTE_DEN_R &= ~0x10;      // 5) disable digital I/O on PE4 PE5
                                  // 5a) configure PE4 as ?? (skip this line because PCTL is for digital only)
  GPIO_PORTE_PCTL_R = GPIO_PORTE_PCTL_R&0xFFF0FFFF;
  GPIO_PORTE_AMSEL_R |= 0x10;     // 6) enable analog functionality on PE4 PE5
	
	GPIO_PORTD_DIR_R &= ~0x04;	// 2) Make pin input
	GPIO_PORTD_AFSEL_R |= 0x04;	// 3) Turn on Alternate function
	GPIO_PORTD_DEN_R &= ~0x04;	// 4) Disable digital
	GPIO_PORTD_PCTL_R = GPIO_PORTD_PCTL_R&0xFFFFF0FF;
	GPIO_PORTD_AMSEL_R |= 0x04;	// 5) Enable analog

	
	
  ADC0_PC_R &= ~0xF;              // 8) clear max sample rate field
  ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;          // 9) Sequencer 3 is lowest priority
  ADC0_ACTSS_R &= ~0x0004;        // 10) disable sample sequencer 2
  ADC0_EMUX_R &= ~0x0F00;         // 11) seq2 is software trigger
  ADC0_SSMUX2_R = 0x0059;         // 12) set channels for SS2
  ADC0_SSCTL2_R = 0x0060;         // 13) no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
  ADC0_IM_R &= ~0x0004;           // 14) disable SS2 interrupts
  ADC0_ACTSS_R |= 0x0004;         // 15) enable sample sequencer 2
	ADC0_SAC_R = 0x06;					// 9) Average of 32 samples.

	
//		ADC0_SSPRI_R = 0x0123;			// 3) Set seq 3 highest priority
//		ADC0_ACTSS_R &= ~0x0008;		// 4) Disable sample sequencer 3
//		ADC0_EMUX_R &= ~0xF000;			// 5) Software trigger
//		ADC0_SSMUX3_R = (ADC0_SSMUX3_R & ~0x0000000F) + 5; //6) Set channel to 5 (PD2)
//		ADC0_SSCTL3_R = 0x06;				// 7) IE0, END0
//		ADC0_IM_R &= ~0x0008;				// 8) Clear bit 3 to disable interrupts for seq 3
//		ADC0_ACTSS_R |= 0x0008;			// 10) Enable sample sequencer 3 again
}

void ADC_Init1(void){
	volatile unsigned long delay;
	SYSCTL_RCGCADC_R |= 0x00000002; // 1) activate ADC1
  SYSCTL_RCGCGPIO_R |= 0x10
	; // 1) activate clock for Port E and Port D
	
	delay = SYSCTL_RCGCGPIO_R;
	delay = SYSCTL_RCGCGPIO_R;
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
	
  GPIO_PORTE_DIR_R &= ~0x28;      // 3) make PE2 PE3 input
  GPIO_PORTE_AFSEL_R |= 0x28;     // 4) enable alternate function on PE2 PE3
  GPIO_PORTE_DEN_R &= ~0x28;      // 5) disable digital I/O on PE2 PE3
	
  GPIO_PORTE_PCTL_R = GPIO_PORTE_PCTL_R&0xFF0F0FFF; // 
  GPIO_PORTE_AMSEL_R |= 0x28;     // 6) enable analog functionality on PE4 PE5	


  ADC1_PC_R &= ~0xF;              // 8) clear max sample rate field
  ADC1_PC_R |= 0x1;               //    configure for 125K samples/sec
  ADC1_SSPRI_R = 0x3210;          // 9) Sequencer 3 is lowest priority
  ADC1_ACTSS_R &= ~0x0004;        // 10) disable sample sequencer 2
  ADC1_EMUX_R &= ~0x0F00;         // 11) seq2 is software trigger
  ADC1_SSMUX2_R = 0x0080;         // 12) set channels for SS2
  ADC1_SSCTL2_R = 0x0060;         // 13) no TS0 D0 IE0 END0 TS1 D1, yes IE1 END1
  ADC1_IM_R &= ~0x0004;           // 14) disable SS2 interrupts
  ADC1_ACTSS_R |= 0x0004;         // 15) enable sample sequencer 2
	ADC1_SAC_R = 0x06;					// 9) Average of 32 samples.

}

//------------ADC_In------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
// measures from PD2, analog channel 5
//uint32_t ADC_In(void){  
//	uint32_t data;
//	
//  ADC0_PSSI_R = 0x0008;            
//  while((ADC0_RIS_R&0x08)==0){};  
//  data = ADC0_SSFIFO3_R&0xFFF;   
//  ADC0_ISC_R = 0x0008;             
//  return data;
//}
void ADC_In(int32_t data[2]){   //PE4 gets put in first -> x    //PD2 is second -> y
  ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
  data[1] = ADC0_SSFIFO2_R&0xFFF;  // 3A) read first result
  data[0] = ADC0_SSFIFO2_R&0xFFF;  // 3B) read second result
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}

void ADC_In1(int32_t data[2]){   //PB5 gets put in first -> x   //PE5 is second -> y
  ADC1_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC1_RIS_R&0x04)==0){};   // 2) wait for conversion done
  data[1] = ADC1_SSFIFO2_R&0xFFF;  // 3A) read first result
  data[0] = ADC1_SSFIFO2_R&0xFFF;  // 3B) read second result
  ADC1_ISC_R = 0x0004;             // 4) acknowledge completion	
	
	
}

