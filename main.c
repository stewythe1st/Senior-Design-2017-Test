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
#include "math.h"
#include "float.h"
#include "FPU.h"

#include "adc_intf.h"
#include "lcd_intf.h"
#include "pie_intf.h"


/***********************************************************
* Compiler Constants
************************************************************/
#define RFFT_STAGES     ( 9 )
#define RFFT_SIZE       ( 1 << RFFT_STAGES ) // 512
#define PI              ( 3.14159 )
#define RESULTS_BUF_SZ  ( RFFT_SIZE )


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

#pragma DATA_SECTION(AdcaRegs,"AdcaRegsFile");
volatile struct ADC_REGS AdcaRegs;                  // ADC A registers

#pragma DATA_SECTION(AdcaResultRegs,"AdcaResultFile");
volatile struct ADC_RESULT_REGS AdcaResultRegs;     // ADC A results register

#pragma DATA_SECTION(EPwm1Regs,"EPwm1RegsFile");
volatile struct EPWM_REGS EPwm1Regs;                // ePWM register

#pragma DATA_SECTION(CpuSysRegs,"CpuSysRegsFile");
volatile struct CPU_SYS_REGS CpuSysRegs;

#pragma DATA_SECTION(PieVectTable,"PieVectTableFile");
volatile struct PIE_VECT_TABLE PieVectTable;

#pragma DATA_SECTION(PieCtrlRegs,"PieCtrlRegsFile");
volatile struct PIE_CTRL_REGS PieCtrlRegs;


/***********************************************************
* Global Variables
************************************************************/
RFFT_F32_STRUCT rfft;
const char * const noteStr[12]  = { "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G" };
char printstr[10];
float32 buf_a[ RESULTS_BUF_SZ ];
float32 buf_b[ RESULTS_BUF_SZ ];
volatile Uint16 bufferFull               = 0;
Uint16 buf_idx                  = 0;
float32* samp_buf                = buf_a;
float32* conv_buf                = buf_b;
float freq, freq2;


/***********************************************************
* Main
************************************************************/
int main( void )
    {
    // Initializes system control, device clock, and peripherals
    Device_init();

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    // Initializes PIE and clear PIE registers. Disables CPU interrupts and clears all CPU interrupt flags
    //Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell interrupt service routines (ISR)
    //Interrupt_initVectorTable();

    // Init GPIO
    GPIO_setDirectionMode( 92, GPIO_DIR_MODE_OUT );

    // Init LCD Module
    lcd_setup();

    // Test LCD Module
    lcd_print( "Hello world!" );

    init_fft();

    // Map ISR function
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    EDIS;

    // Init ADC Modules
    ConfigureADC();
    ConfigureEPWM();
    SetupADCEpwm(0);

    // Enable global Interrupts and higher priority real-time debug events:
    IER |= 0x0001; //Enable group 1 interrupts
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // enable PIE interrupt
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // sync ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    // Start ePWM
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0; // unfreeze, and enter up count mode

    freq = 0;
    freq2 = -1;
    for(;;)
        {

        // Calculate frequency
        freq = calc_fft();

        // Convert frequency to scale degree
        // https://en.wikipedia.org/wiki/Piano_key_frequencies
        Uint16 note = round( 12 * log2( freq / 440 ) + 49 );

        // Update LCD display
        if( freq2 != freq )
            {
            lcd_clear();
            if( freq < 50 ) // any frequency below 50Hz is probably just no signal
                {
                lcd_print( "No signal!" );
                }
            else
                {
                itoa( (Uint16)freq, printstr, 10 );
                lcd_print( printstr );
                //lcd_print( (char * )( noteStr[ note % 12 ] ), 2 );
                }
            }

        // Wait for ADC ISR to fill buffer
        while( bufferFull != 1 ){;}
        bufferFull = 0;
        freq2 = freq;

        }
    }


/***********************************************************
* Initialize FFT
************************************************************/
void init_fft()
    {
    // Declarations
    Uint16  i;

    // Clear input buffers:
    for( i = 0; i < RFFT_SIZE; i++ )
        {
        RFFTin1Buff[i] = 0.0f;
        }

    rfft.FFTSize   = RFFT_SIZE;
    rfft.FFTStages = RFFT_STAGES;
    rfft.InBuf     = &RFFTin1Buff[ 0 ];  // Input buffer
    rfft.OutBuf    = &RFFToutBuff[ 0 ];  // Output buffer
    rfft.CosSinBuf = &RFFTF32Coef[ 0 ];  // Twiddle factor buffer
    rfft.MagBuf    = &RFFTmagBuff[ 0 ];  // Magnitude buffer

    // Calculate twiddle factor
    RFFT_f32_sincostable( &rfft );
    }

/***********************************************************
* Calculate FFT
************************************************************/
Uint16 calc_fft()
    {
    // Declarations
    Uint16  i;
    Uint32  fo;
    Uint16 zero_cnt = 0;

    // Clean up buffers
    for( i = 0; i < RFFT_SIZE; i++ )
        {
        RFFToutBuff[ i ] = 0;
        RFFTin1Buff[ i ] = conv_buf[ i ];
        if( conv_buf[ i ] == 0)
            {
            zero_cnt++;
            }
        }

    // If we receive more than 3/4 zeros, its probably just no signal
    if(zero_cnt > ( 3 * RFFT_SIZE / 4 ) )
        {
        return 0;
        }

    // Clean up magnitude buffer
    for( i = 0; i < RFFT_SIZE / 2; i++ )
        {
        RFFTmagBuff[i] = 0;
        }

    // Calculate real FFT
    rfft.InBuf = conv_buf;
    RFFT_f32( &rfft );

    // Calculate magnitude
    RFFT_f32_mag( &rfft );

    // Find peak
    Uint16 max = 1;
    for( i = max; i < ( ( RFFT_SIZE / 2 ) - 1); i++ )
        {
        if( RFFTmagBuff[ i ] > RFFTmagBuff[ max ] )
            {
            max = i;
            }
        }

    // Get output frequency
    fo = (Uint32)SAMP_FREQ * max / ( RFFT_SIZE / 2);

    return fo;
    }


/***********************************************************
* ADC1 Interrupt Service Routine
************************************************************/
Uint16 toggle = 0;
interrupt void adca1_isr(void)
    {
    // Toggle GPIO pin to verify interrupt frequency
    toggle++;
    if(toggle % 2 == 0)
        GPIO_writePin( 92, 1 );
    else
        GPIO_writePin( 92, 0 );

    // Get input from ADC
    samp_buf[ buf_idx ] = ( float32 )AdcaResultRegs.ADCRESULT0;
    //samp_buf[ buf_idx ] = sin( buf_idx * PI * 440 / SAMP_FREQ );

    // Did we fill the buffer yet?
    buf_idx++;
    if( buf_idx >= RESULTS_BUF_SZ )
        {
        buf_idx     = 0;
        bufferFull  = 1;
        // Swap buffers for conversion
        if( samp_buf == buf_a )
            {
            samp_buf = buf_b;
            conv_buf = buf_a;
            }
        else
            {
            samp_buf = buf_a;
            conv_buf = buf_b;
            }
        }

    // Clear interrupt flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = 0x0001;
    }

/***********************************************************
* ITOA
************************************************************/
void itoa(long unsigned int value, char* result, int base)
    {
      // check that the base if valid
      if (base < 2 || base > 36) { *result = '\0';}

      char* ptr = result, *ptr1 = result, tmp_char;
      int tmp_value;

      do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
      } while ( value );

      // Apply negative sign
      if (tmp_value < 0) *ptr++ = '-';
      *ptr-- = '\0';
      while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
      }

    }
