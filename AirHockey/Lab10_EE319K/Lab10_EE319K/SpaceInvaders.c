// SpaceInvaders.c
// Runs on TM4C123
// Jonathan Valvano and Daniel Valvano
// This is a starter project for the EE319K Lab 10

// Last Modified: 1/12/2022 
// http://www.spaceinvaders.de/
// sounds at http://www.classicgaming.cc/classics/spaceinvaders/sounds.php
// http://www.classicgaming.cc/classics/spaceinvaders/playguide.php

// ******* Possible Hardware I/O connections*******************
// Slide pot pin 1 connected to ground
// Slide pot pin 2 connected to PD2/AIN5
// Slide pot pin 3 connected to +3.3V 
// buttons connected to PE0-PE3
// 32*R resistor DAC bit 0 on PB0 (least significant bit)
// 16*R resistor DAC bit 1 on PB1
// 8*R resistor DAC bit 2 on PB2 
// 4*R resistor DAC bit 3 on PB3
// 2*R resistor DAC bit 4 on PB4
// 1*R resistor DAC bit 5 on PB5 (most significant bit)
// LED on PD1
// LED on PD0


#include <stdint.h>
#include "../../inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "Print.h"
#include "Random.h"
#include "TExaS.h"
#include "ADC.h"
#include "Images.h"
#include "Sound.h"
#include "Timer1.h"
#include "DAC.h"

void MalletMovementx(void);
void MalletMovementy(void);
void movePuck();
void MalletMovementx2(void);
void MalletMovementy2(void);
void botmovement(void);

int red_score = 0;
int green_score = 0;

uint32_t player1[2] = {0};
uint32_t player2[2] = {0};

int flag_redraw = 0;

int playerselectionflag;

char score_red = 0;
char score_green = 0;

	uint8_t status;

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Delay100ms(uint32_t count); // time delay in 0.1 seconds
void Delay10ms(uint32_t count); // time delay in 0.01 seconds
void Wait1ms(uint32_t count); // time delay in 0.001 seconds
#define Englishstart 100
#define Spanishstart 200
#define multiplayerflag 27
#define singleplayerflag 43

//********************************************************************************
// debuging profile, pick up to 7 unused bits and send to Logic Analyzer
#define PB54                  (*((volatile uint32_t *)0x400050C0)) // bits 5-4
#define PF321                 (*((volatile uint32_t *)0x40025038)) // bits 3-1
// use for debugging profile
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PB5       (*((volatile uint32_t *)0x40005080)) 
#define PB4       (*((volatile uint32_t *)0x40005040)) 

// TExaSdisplay logic analyzer shows 7 bits 0,PB5,PB4,PF3,PF2,PF1,0 
// edit this to output which pins you use for profiling
// you can output up to 7 pins
void LogicAnalyzerTask(void){
  UART0_DR_R = 0x80|PF321|PB54; // sends at 10kHz
}
void ScopeTask(void){  // called 10k/sec
  UART0_DR_R = (ADC1_SSFIFO3_R>>4); // send ADC to TExaSdisplay
}
// edit this to initialize which pins you use for profiling
void Profile_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x30;      // activate port B,F,E
  while((SYSCTL_PRGPIO_R&0x20) != 0x20){};
  GPIO_PORTF_DIR_R |=  0x0E;   // output on PF3,2,1 
  GPIO_PORTF_DEN_R |=  0x0E;   // enable digital I/O on PF3,2,1
  //GPIO_PORTB_DIR_R |=  0x30;   // output on PB4 PB5
  //GPIO_PORTB_DEN_R |=  0x30;   // enable on PB4 PB5  
	GPIO_PORTE_DIR_R &= ~0x0F;   // input on PE 3-0
	GPIO_PORTE_DEN_R |= 0x0F;    // enable PE3-PE0
}
//********************************************************************************


typedef void (*handler)(void);

static handler PeriodicTask;   // user function

// ***************** Timer3_Init ****************
// Activate Timer3 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer3_Init(handler task, unsigned long period){
  SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
  PeriodicTask = task;          // user function
  TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER3_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAILR_R = period-1;    // 4) reload value
  TIMER3_TAPR_R = 0;            // 5) bus clock resolution
  TIMER3_ICR_R = 0x00000001;    // 6) clear TIMER3A timeout flag
  TIMER3_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
  TIMER3_CTL_R = 0x00000001;    // 10) enable TIMER3A
}

void Timer3A_Handler(void){
		TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
		(*PeriodicTask)();                // execute user task
}

void game_engine() {
			ADC_In(player1);
	//ADC_In1(player2);
	MalletMovementx();     //need to put black image
	//MalletMovementy();     //need to put black image
	MalletMovementx2();
	//MalletMovementy2();
	movePuck();
	flag_redraw = 1;
}


void SysTick_Init(void){ 
	DAC_Init();
	NVIC_ST_CTRL_R = 0; 
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x40000000;   //this has higher priority than the game engine timer
  NVIC_ST_RELOAD_R = 0x00FFFFFF; 
  NVIC_ST_CURRENT_R = 0; 
  NVIC_ST_CTRL_R = 0x00000005; 
}

int tickCounter;

void SysTick_Handler(void){
	DAC_Out(sounds[sound_atm].soundpt[indexx]);
	indexx++;
	if( indexx >=(sounds[sound_atm].length)){
		Sound_Off();
	}
}


void Timer1A_Handler(void){ // can be used to perform tasks in background
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER1A timeout
	ADC_In(player1);
	//ADC_In1(player2);
	MalletMovementx();     //need to put black image
	//MalletMovementy();     //need to put black image
	MalletMovementx2();
	//MalletMovementy2();
	movePuck();
	flag_redraw = 1;

	
}

int flagstart = 0;
void singleordouble(){
	ST7735_FillScreen(0x0000);       // set screen to black
	if(flagstart == Spanishstart){
		while(1){
			ST7735_DrawBitmap (38,90, ENG, 64,24);
      ST7735_DrawBitmap(0, 159, BottomL, 46,2); // player ship bottom
			ST7735_DrawBitmap(82, 159, BottomR, 46,2); // player ship bottom
			ST7735_DrawBitmap(0, 1, TopL, 46,2); // player ship bottom
			ST7735_DrawBitmap(82, 1, TopR, 46,2); // player ship bottom
			if(((GPIO_PORTE_DATA_R&0x01)==1)){
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (48,88, three, 20,16);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,92, two, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,95, one, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000);       // set screen to black
					playerselectionflag = singleplayerflag;
					break;
				}
				if(((GPIO_PORTE_DATA_R&0x02)==2)){
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (48,88, three, 20,16);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,92, two, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,95, one, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000);       // set screen to black
					playerselectionflag = multiplayerflag;
					break;
				}
				
		}
	}
		if(flagstart == Englishstart){
		while(1){
			ST7735_DrawBitmap (38,90, SPAN, 64,24);
      ST7735_DrawBitmap(0, 159, BottomL, 46,2); // player ship bottom
			ST7735_DrawBitmap(82, 159, BottomR, 46,2); // player ship bottom
			ST7735_DrawBitmap(0, 1, TopL, 46,2); // player ship bottom
			ST7735_DrawBitmap(82, 1, TopR, 46,2); // player ship bottom
				if(((GPIO_PORTE_DATA_R&0x01)==1)){
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (48,88, three, 20,16);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,92, two, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,95, one, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000);       // set screen to black
					playerselectionflag = multiplayerflag;
					break;
				}
				if(((GPIO_PORTE_DATA_R&0x02)==2)){
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (48,88, three, 20,16);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,92, two, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000); 
					ST7735_DrawBitmap (50,95, one, 24,24);
					Delay100ms(10);
					ST7735_FillScreen(0x0000);       // set screen to black
					playerselectionflag = multiplayerflag;
					break;
				}
		}
		}
}

void start_Screen(){
	ST7735_FillScreen(0x0000);       // set screen to black
		ST7735_SetCursor(5,7);
		ST7735_OutString("Air Hockey");
	while(1){
    ST7735_DrawBitmap(0, 159, BottomL, 46,2); // player ship bottom
    ST7735_DrawBitmap(82, 159, BottomR, 46,2); // player ship bottom
    ST7735_DrawBitmap(0, 1, TopL, 46,2); // player ship bottom
    ST7735_DrawBitmap(82, 1, TopR, 46,2); // player ship bottom
    ST7735_DrawBitmap(76, 54, MalletG, 16,16);
    ST7735_DrawBitmap(36, 120, MalletR, 16,16);
		if(((GPIO_PORTE_DATA_R&0x01)==1)){
			flagstart = Englishstart;
			singleordouble();
			break;
		}
		if(((GPIO_PORTE_DATA_R&0x02)==2)){
			flagstart = Spanishstart;
		singleordouble();
		break;
	}
}
}




// You can't use this timer, it is here for starter code only 
// you must use interrupts to perform delays
void Delay100ms(uint32_t count){uint32_t volatile time;
  while(count>0){
    time = 727240;  // 0.1sec at 80 MHz
    while(time){
      time--;
    }
    count--;
  }
}


uint32_t semaphore = 0;	//getting adc data flag
uint32_t ADCdata;
uint32_t scoregreen = 0;
uint32_t scorered = 0;

#define stop 0
#define moved 1
#define scoregreen 10
#define scorered 15

int Collision(int32_t x1, int32_t y1, int32_t x2, int32_t y2){
	return ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
}

struct GreenMallet{
	int8_t x;
	int8_t y;
	int8_t xold, yold;
	int8_t dx;
	int8_t dy;
	uint16_t w, h;
	const uint16_t *image;
	uint8_t status;
};
typedef struct GreenMallet Gm_type;

Gm_type MalletGreen = {64, 16, 64, 16, 0, 0, 16, 16, MalletG};//insert position and image

struct RedMallet{
	uint8_t x;
	uint8_t y;
	uint8_t xold, yold;
	uint8_t dx;
	uint8_t dy;
	uint16_t w, h;
	const uint16_t *image;
	uint8_t status;
};	
typedef struct RedMallet Rm_type;
Rm_type MalletRed = {64, 159, 64, 159, 0, 0, 16, 16, MalletR};//insert position and image

void MalletMovementx(void){
	int xPos = player1[1];
	int32_t desired = (111*xPos)/4095;
	int32_t error = desired - MalletGreen.x;
	if(error > 0) {
		MalletGreen.dx = 2;
	}
	else if(error < 0){
		MalletGreen.dx = -2;
	}
	
	MalletGreen.x += MalletGreen.dx;
}

void MalletMovementy(void){
	int yPos = player1[0];
	int32_t desired = (60*yPos)/4095;
	int32_t error = desired - MalletGreen.y;
	if(error > 0) {
		MalletGreen.dy = 2;
	}
	else if(error < 0){
		MalletGreen.dy = -2;
	}
	
	MalletGreen.y += MalletGreen.dy;
	
}

void MalletMovementx2(void){ //check
	int xPos = 4095 - player1[0];
	int32_t desired = (109*xPos)/4095;
	int32_t error = desired - MalletRed.x;
	if(MalletRed.x <= 5) {
		MalletRed.dx = 0;
	}
	else if(MalletRed.x >= 108) {
		MalletRed.dx = 0;
	}
	if(error > 0) {
		MalletRed.dx = 2;
	}
	else if(error < 0){
		MalletRed.dx = -2;
	}
	
	MalletRed.x += MalletRed.dx;
}

void MalletMovementy2(void){ //check
	int yPos = player2[0];
	int32_t desired = (60*yPos)/4095;
	int32_t error = desired - MalletRed.y;
	if(error > 0) {
		MalletRed.dy = 2;
	}
	else if(error < 0){
		MalletRed.dy = -2;
	}
	
	MalletRed.y += MalletRed.dy;
	
}

struct Puck{
	uint8_t x;
	uint8_t y;
	uint8_t xold, yold;
	uint8_t dx;
	uint8_t dy;
	uint16_t w, h;
	const uint16_t *image;
	uint8_t status;
};
typedef struct Puck P_type;
P_type Puckpos = {64, 80, 64, 80, 1, 1, 8, 8, PuckImage};//insert position and image

void movePuck(){
	Puckpos.xold = Puckpos.x;	
	Puckpos.yold = Puckpos.y;	
	Puckpos.x += Puckpos.dx;
	Puckpos.y += Puckpos.dy;
	
	if(((GPIO_PORTE_DATA_R&0x01)==1)){
	Puckpos.xold = Puckpos.x;	
	Puckpos.yold = Puckpos.y;	
	Puckpos.x += Puckpos.dx;
	Puckpos.y += Puckpos.dy;
	}
	
	if(((GPIO_PORTE_DATA_R&0x02)==2)){
	Puckpos.xold = Puckpos.x;	
	Puckpos.yold = Puckpos.y;	
	Puckpos.x -= Puckpos.dx;
	Puckpos.y -= Puckpos.dy;	
	}	
	
	if((MalletRed.y - MalletRed.h == Puckpos.y) && (Puckpos.x - MalletRed.x <= MalletRed.w) && (Puckpos.x - MalletRed.x >= (-Puckpos.w))){
		Puckpos.dy *=-1;
	  Sound_Start(0);
}
	else if(((MalletRed.x - Puckpos.w) == Puckpos.x) && (Puckpos.y > (MalletRed.y - MalletRed.h))){
		Puckpos.dx *=-1;
	  Sound_Start(0);

	}
	else if(((MalletRed.x + MalletRed.w == Puckpos.x) && (Puckpos.y > (MalletRed.y - MalletRed.h)))){
		Puckpos.dx *=-1;
	  Sound_Start(0);

	}
	
	if(((Puckpos.y - Puckpos.h) == MalletGreen.y) && (Puckpos.x - MalletRed.x) <= MalletGreen.w && ((Puckpos.x - MalletGreen.x) >= (-Puckpos.w))){
		Puckpos.dy *=-1;
	  Sound_Start(0);

	}
	else if(((MalletGreen.x - Puckpos.w) == Puckpos.x) && ((Puckpos.y - Puckpos.h) < MalletGreen.y)){
		Puckpos.dx *=-1;
	  Sound_Start(0);

	}
	else if(((MalletGreen.x + MalletGreen.w) == Puckpos.x) && ((Puckpos.y - Puckpos.h) < MalletGreen.y)){
		Puckpos.dx *=-1;
	  Sound_Start(0);

	}
	
	else if( Puckpos.x == 0 ) {
		Puckpos.dx *= -1;
	  Sound_Start(0);

	}
	else if( Puckpos.x == 119) {
		Puckpos.dx *= -1;
	  Sound_Start(0);

	}
	else if( Puckpos.y == 159) {
			  Sound_Start(1);

		green_score ++;
		Puckpos.x = 64;
		Puckpos.y = 80;
		ST7735_FillRect(Puckpos.xold, Puckpos.yold - Puckpos.h + 1, Puckpos.w, Puckpos.h, ST7735_BLACK);
	}
	else if( Puckpos.y == 8) {
			  Sound_Start(1);

		red_score++;
		Puckpos.x = 64;
		Puckpos.y = 80;
		ST7735_FillRect(Puckpos.xold, Puckpos.yold - Puckpos.h + 1, Puckpos.w, Puckpos.h, ST7735_BLACK);
	}
}

void InitialPosition(void){
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawBitmap(MalletGreen.x, MalletGreen.y, MalletGreen.image, MalletGreen.w, MalletGreen.h);
	MalletGreen.status = stop;
	ST7735_DrawBitmap(MalletRed.x, MalletRed.y, MalletRed.image, MalletRed.w, MalletRed.h);
	MalletRed.status = stop;
	ST7735_DrawBitmap(Puckpos.x, Puckpos.y, Puckpos.image, Puckpos.w, Puckpos.h);
	Puckpos.status = stop;
}

int main(void){ 
  DisableInterrupts();
  TExaS_Init(NONE);       // Bus clock is 80 MHz 
  Output_Init();
	SysTick_Init();
	ADC_Init();
	//ADC_Init1();
	Profile_Init();
	//start_Screen();
	//Timer1_Init(80000000/30, 4);
	Timer3_Init(game_engine, 80000000/30);
	InitialPosition();
	start_Screen();
	EnableInterrupts();

	while(1) {
	
		if(flag_redraw) {
			ST7735_DrawBitmap(MalletGreen.x, MalletGreen.y, MalletG, MalletGreen.w, MalletGreen.h);
			ST7735_DrawBitmap(MalletRed.x, MalletRed.y, MalletR, MalletRed.w, MalletRed.h);
			ST7735_DrawBitmap(Puckpos.x, Puckpos.y, PuckImage, Puckpos.w, Puckpos.h);
			flag_redraw = 0;
		
		}
		if(green_score == 7) {
			ST7735_FillScreen(ST7735_BLACK);
			ST7735_SetCursor(0, 0);
			if(flagstart == Englishstart){
			ST7735_OutString("Verde Gana!");
			ST7735_SetCursor(1,1);
			ST7735_OutString("Puntaje: 7");
			}
			else if(flagstart == Spanishstart){
			ST7735_OutString("Green Wins! Score: 7");	
			}
			for(volatile int delay = 0; delay < 8000000; delay++);
			break;
		}
		if(red_score == 7) { 
			ST7735_FillScreen(ST7735_BLACK);
			ST7735_SetCursor(0, 0);
			if(flagstart == Englishstart){
				ST7735_OutString("Rojo Gana!");
			ST7735_SetCursor(1,1);
				ST7735_OutString("Puntaje: 7");
			}
			else if(flagstart == Spanishstart){
			ST7735_OutString("Red Wins! Score: 7");
			}
			for(volatile int delay = 0; delay < 8000000; delay++);
			break;
		}

	}
		ST7735_FillScreen(ST7735_BLACK);
	  ST7735_SetCursor(4,7);
		if(flagstart == Spanishstart){
			ST7735_OutString("Game Over!");	
		}
		if(flagstart == Englishstart){
			ST7735_OutString("Juego Terminado!");			
		}
			for(volatile int delay = 0; delay < 8000000; delay++);


}

