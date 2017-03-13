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


/***********************************************************
* Headers
************************************************************/
#include "adc_intf.h"


/***********************************************************
* Global Variables
************************************************************/


/***********************************************************
* Configure ADC
************************************************************/
void ConfigureADC(void)
    {
    EALLOW;                                     // allow writing to protected registers
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // set ADCCLK divider to /4
    AdcSetMode( ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE );
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // set pulse position to late
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // power up the ADC
    EDIS;                                       // allow writing to protected registers

    // 1ms delay for powerup time
    volatile uint32_t j;
    for(j=10000; j>0; j--);
    //DELAY_US(1000);
    }

/***********************************************************
* Setup for EPWM trigger
************************************************************/
void SetupADCEpwm(Uint16 channel)
{
    // determine minimum acquisition window (in SYSCLKS) based on resolution
    Uint16 acqps;
    if( ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION )
        {
        acqps = 14; // 75ns
        }
    else // resolution is 16-bit
        {
        acqps = 63; // 320ns
        }
    EALLOW;                                     // allow writing to protected registers
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;    // SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;      // sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;        // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared
    EDIS;                                       // disallow writing to protected registers
}

/***********************************************************
* Setup for software trigger
************************************************************/
void SetupADCSoftware(void)
    {
    // Set minimum acquisition window (in SYSCLKS) based on resolution
    Uint16 acqps;
    if( ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION )
        {
        acqps = 14; // 75ns
        }
    else //resolution is 16-bit
        {
        acqps = 63; // 320ns
        }

    EALLOW;                                 // allow writing to protected registers
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;      // SOC0 will convert pin A0 (pin 27 on our launchpad)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;  // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  // end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // make sure INT1 flag is cleared
    EDIS;                                   // disallow writing to protected registers
    }


/***********************************************************
* ConfigureEPWM - Configure EPWM SOC and compare values
************************************************************/
void ConfigureEPWM(void)
{
    EALLOW;                                 // allow writing to protected registers
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;       // enable epwm clock
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;         // disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;        // select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;         // generate pulse on 1st event
    EPwm1Regs.CMPA.bit.CMPA = 0x0800;       // set compare A value to 2048 counts
    EPwm1Regs.TBPRD = EPWM_CNTS;            // set period (configure in adc_intf.h)
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;        // freeze counter
    EDIS;                                   // disallow writing to protected registers
}

/***********************************************************
* Set the resolution and signalmode for a given ADC. This will
* ensure that the correct trim is loaded.
************************************************************/
void AdcSetMode( Uint16 adc, Uint16 resolution, Uint16 signalmode )
    {
    Uint16 adcOffsetTrimOTPIndex;           // index into OTP table of ADC offset trims
    Uint16 adcOffsetTrim;                   // temporary ADC offset trim

    // Re-populate INL trim
    CalAdcINL( adc );

    if( 0xFFFF != *( ( Uint16* )GetAdcOffsetTrimOTP ) )
    {
        //
        //offset trim function is programmed into OTP, so call it
        //

        //
        //calculate the index into OTP table of offset trims and call
        //function to return the correct offset trim
        //
        adcOffsetTrimOTPIndex = 4*adc + 2*resolution + 1*signalmode;
        adcOffsetTrim = (*GetAdcOffsetTrimOTP)(adcOffsetTrimOTPIndex);
    }
    else
    {
        //
        //offset trim function is not populated, so set offset trim to 0
        //
        adcOffsetTrim = 0;
    }

    //
    //Apply the resolution and signalmode to the specified ADC.
    //Also apply the offset trim and, if needed, linearity trim correction.
    //
    switch(adc)
    {
        case ADC_ADCA:
            AdcaRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcaRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcaRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdcaRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcaRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        /*
        case ADC_ADCB:
            AdcbRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcbRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcbRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdcbRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcbRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCC:
            AdccRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdccRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdccRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdccRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdccRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        case ADC_ADCD:
            AdcdRegs.ADCCTL2.bit.RESOLUTION = resolution;
            AdcdRegs.ADCCTL2.bit.SIGNALMODE = signalmode;
            AdcdRegs.ADCOFFTRIM.all = adcOffsetTrim;
            if(ADC_RESOLUTION_12BIT == resolution)
            {
                //
                //12-bit linearity trim workaround
                //
                AdcdRegs.ADCINLTRIM1 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM2 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM4 &= 0xFFFF0000;
                AdcdRegs.ADCINLTRIM5 &= 0xFFFF0000;
            }
        break;
        */
    }
}

//
// CalAdcINL - Loads INL trim values from OTP into the trim registers of the
//             specified ADC. Use only as part of AdcSetMode function, since
//             linearity trim correction is needed for some modes.
//
void CalAdcINL(Uint16 adc)
{
    switch(adc)
    {
        case ADC_ADCA:
            if(0xFFFF != *((Uint16*)CalAdcaINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcaINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCB:
            if(0xFFFF != *((Uint16*)CalAdcbINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcbINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCC:
            if(0xFFFF != *((Uint16*)CalAdccINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdccINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
        case ADC_ADCD:
            if(0xFFFF != *((Uint16*)CalAdcdINL))
            {
                //
                //trim function is programmed into OTP, so call it
                //
                (*CalAdcdINL)();
            }
            else
            {
                //
                //do nothing, no INL trim function populated
                //
            }
            break;
    }
}

