/************************************************************************************************************
* LCD Interface
*
* Author:
* Stuart Miller
* Missouri University of Science & Technology
* Computer Engineering
* 2017
*
************************************************************************************************************/
#ifndef _MAIN_H_
#define _MAIN_H_


/***********************************************************
* Headers
************************************************************/
#include "FPU.h"
#include "F2837xS_adc.h"
#include "F2837xS_epwm.h"
#include "F2837xS_sysctrl.h"
#include "F2837xS_pievect.h"
#include "F2837xS_piectrl.h"


/***********************************************************
* Compiler Constants
************************************************************/
#define CPU_RATE   5.00L        // for a 200MHz CPU clock speed (SYSCLKOUT)
#define SYSCLK     200000000    // for a 200MHz CPU clock speed (SYSCLKOUT)


/***********************************************************
* Functions
*************************************************************/
void init_fft();
Uint16 calc_fft();
interrupt void adca1_isr( void );
void itoa(long unsigned int value, char* result, int base);
extern void F28x_usDelay( long LoopCount );
#define DELAY_US( A )  F28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)



#endif

