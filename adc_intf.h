/************************************************************************************************************
* ADC Interface
*
* Author:
* Stuart Miller
* Missouri University of Science & Technology
* Computer Engineering
* 2017
*
************************************************************************************************************/
#ifndef _ADC_H_
#define _ADC_H_


/***********************************************************
* Headers
************************************************************/
#include "driverlib.h"
#include "device.h"
#include "main.h"


/***********************************************************
* Compiler Constants
************************************************************/
// Interrupt frequency (for sampling)
#define SAMP_FREQ ( 10000 )     // 10kHz
#define EPWM_CNTS ( SYSCLK / SAMP_FREQ / 4 )

// Definitions for specifying an ADC
#define ADC_ADCA 0
#define ADC_ADCB 1
#define ADC_ADCC 2
#define ADC_ADCD 3

// Definitions for selecting ADC resolution
#define ADC_RESOLUTION_12BIT 0
#define ADC_RESOLUTION_16BIT 1

// Definitions for selecting ADC signal mode
// (single-ended mode is only a valid mode for 12-bit resolution)
#define ADC_SIGNALMODE_SINGLE 0
#define ADC_SIGNALMODE_DIFFERENTIAL 1

// The following pointers to functions calibrate the ADC linearity.  Use this
// in the AdcSetMode(...) function only
#define CalAdcaINL (void   (*)(void))0x0703B4
#define CalAdcbINL (void   (*)(void))0x0703B2
#define CalAdccINL (void   (*)(void))0x0703B0
#define CalAdcdINL (void   (*)(void))0x0703AE

// The following pointer to a function call looks up the ADC offset trim for a
// given condition. Use this in the AdcSetMode(...) function only.
#define GetAdcOffsetTrimOTP (Uint16 (*)(Uint16 OTPoffset))0x0703AC

/***********************************************************
* ADC Functions
************************************************************/
interrupt void adca1_isr( void );
void ConfigureADC( void );
void SetupADCSoftware( void );
void SetupADCEpwm(Uint16 channel);
void AdcSetMode( Uint16 adc, Uint16 resolution, Uint16 signalmode );
void CalAdcINL( Uint16 adc );
void ConfigureEPWM( void );

#endif
