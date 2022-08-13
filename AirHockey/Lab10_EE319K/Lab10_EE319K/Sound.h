// Sound.h
// Runs on TM4C123 or LM4F120
// Prototypes for basic functions to play sounds from the
// original Space Invaders.
// Jonathan Valvano
// 11/15/2021 
#ifndef SOUND_H
#define SOUND_H
#include <stdint.h>

void Sound_Init(void);

//******* Sound_Start ************
// This function does not output to the DAC. 
// Rather, it sets a pointer and counter, and then enables the timer interrupt.
// It starts the sound, and the timer ISR does the output
// feel free to change the parameters
// Input: pt is a pointer to an array of DAC outputs
//        count is the length of the array
// Output: none
// special cases: as you wish to implement
void Sound_Start(int soundvalue);

typedef struct{
	const uint8_t *soundpt;
	uint32_t length;
} soundptr_t;


extern int done_sound;
extern const uint8_t sinWave[64];
extern const uint8_t shoot[4080];
extern int sound_atm;
extern uint32_t indexx;
extern soundptr_t sounds[2];


void Sound_Shoot(void);
void Sound_Killed(void);
void Sound_Explosion(void);

void Sound_Fastinvader1(void);
void Sound_Fastinvader2(void);
void Sound_Fastinvader3(void);
void Sound_Fastinvader4(void);
void Sound_Highpitch(void);
void Sound_Off(void);

#endif
