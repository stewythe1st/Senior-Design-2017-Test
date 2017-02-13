/************************************************************************************************************
* Fast Fourier Transform Testing
*
* Author:
* Stuart Miller
* Missouri University of Science & Technology
* Computer Engineering
* 2017
*
* Target:
* TI C2000 Lauchpad
* TMS320F28377S
*
* Adapted from:
* Test_FPU_RFFTF32.c
* DSP2833x Device FIR Test Program
* C28x Floating Point Unit Library V1.31
*
* License:
* This software is licensed for use with Texas Instruments C28x
* family DSCs.  This license was provided to you prior to installing
* the software.  You may review this license by consulting a copy of
* the agreement in the doc directory of this library.
*
************************************************************************************************************/


/***********************************************************
* Headers
************************************************************/
#include "driverlib.h"
#include "device.h"
#include "lcd.h"
#include "math.h"
#include "float.h"
#include "FPU.h"


/***********************************************************
* Compiler Constants
************************************************************/
#define RFFT_STAGES     ( 9 )
#define RFFT_SIZE       ( 1 << RFFT_STAGES ) // 512
#define PI              ( 3.14159 )


/***********************************************************
* Memory Allocations
************************************************************/
#pragma DATA_SECTION( RFFTin1Buff, "RFFTdata1" );   // Buffer alignment for the input array,
float32 RFFTin1Buff[ RFFT_SIZE ];                   // RFFT_f32u(optional), RFFT_f32(required)
                                                    // Output of FFT overwrites input if
                                                    // RFFT_STAGES is ODD
#pragma DATA_SECTION( RFFToutBuff, "RFFTdata2" );
float32 RFFToutBuff[RFFT_SIZE ];                    // Output of FFT here if RFFT_STAGES is EVEN

#pragma DATA_SECTION( RFFTmagBuff, "RFFTdata3" );
float32 RFFTmagBuff[ RFFT_SIZE / 2 + 1 ];           // Additional Buffer used in Magnitude calc

#pragma DATA_SECTION( RFFTF32Coef, "RFFTdata4" );
float32 RFFTF32Coef[ RFFT_SIZE ];                   // Twiddle buffer


/***********************************************************
* Global Variables
************************************************************/
RFFT_F32_STRUCT rfft;
const char * const noteStr[12] = { "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G" };


/***********************************************************
* Main
************************************************************/
int main( void )
    {
	
    // Declarations
    Uint16  i;  // counter var

    // Initializes system control, device clock, and peripherals
    Device_init();

    // Initializes PIE and clear PIE registers. Disables CPU interrupts and clears all CPU interrupt flags
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell interrupt service routines (ISR)
    Interrupt_initVectorTable();

    // Init LCD Module
    lcd_setup();

    // Test LCD Module
    lcd_print( "Hello world!", 12 );

    // Test GPIO pins
    GPIO_setDirectionMode( 92, GPIO_DIR_MODE_OUT );
    GPIO_writePin( 92, 1 );

    // Clear input buffers:
    for( i = 0; i < RFFT_SIZE; i++ )
        {
        RFFTin1Buff[i] = 0.0f;
        }

    // Generate test waveform
    float f = 40;
    float fs = 10000;
    for( i = 0; i < RFFT_SIZE; i++ )
        {
        RFFTin1Buff[ i ] = sin( i * PI * f / fs );
        }

    rfft.FFTSize   = RFFT_SIZE;
    rfft.FFTStages = RFFT_STAGES;
    rfft.InBuf     = &RFFTin1Buff[ 0 ];  // Input buffer
    rfft.OutBuf    = &RFFToutBuff[ 0 ];  // Output buffer
    rfft.CosSinBuf = &RFFTF32Coef[ 0 ];  // Twiddle factor buffer
    rfft.MagBuf    = &RFFTmagBuff[ 0 ];  // Magnitude buffer

    // Calculate twiddle factor
    RFFT_f32_sincostable( &rfft );

    // Clean up output buffer
    for( i = 0; i < RFFT_SIZE; i++ )
        {
        RFFToutBuff[i] = 0;
        }

    // Clean up magnitude buffer
    for( i = 0; i < RFFT_SIZE / 2; i++ )
        {
        RFFTmagBuff[i] = 0;
        }

    // Calculate real FFT
    RFFT_f32( &rfft );

    // Calculate magnitude
    RFFT_f32_mag( &rfft );

    // Find peak
    Uint16 max = 0;
    for( i = 0; i < RFFT_SIZE / 2; i++ )
        {
        if( RFFTmagBuff[ i ] > RFFTmagBuff[ max ] )
            {
            max = i;
            }
        }

    // Get output frequency
    float fo = fs * max / ( RFFT_SIZE / 2);

    // Frequency to scale degree conversion
    // https://en.wikipedia.org/wiki/Piano_key_frequencies
    Uint16 note = round( 12 * log2( fo / 440 ) + 49 );

    lcd_clear();
    lcd_print( noteStr[ note % 12 ], 2 );

    // Just sit and loop forever when done
    for(;;);

    }
