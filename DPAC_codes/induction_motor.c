//###########################################################################
//
// FILE:   Example_2806xAdcSoc.c
//
// TITLE:  ADC Start of Conversion Example
//
//! \addtogroup f2806x_example_list
//! <h1> ADC Start of Conversion (adc_soc)</h1>
//!
//! This ADC example uses ePWM1 to generate a periodic ADC SOC - ADCINT1.
//! Two channels are converted, ADCINA4 and ADCINA2.
//!
//! \b Watch \b Variables \n
//! - x[10]    - Last 10 ADCRESULT0 values
//! - x[10]    - Last 10 ADCRESULT1 values
//! - Ch_sel - Current result number 0-9
//! - LoopCount       - Idle loop counter
//
//###########################################################################
// $TI Release: F2806x Support Library v2.04.00.00 $
// $Release Date: Thu Oct 18 15:47:20 CDT 2018 $
// $Copyright:
// Copyright (C) 2009-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

//
// Function Prototypes
//
__interrupt void adc_isr(void);
void Adc_Config(void);

void spi_xmit(Uint16 a);
void spi_fifo_init(void);
void spi_init(void);
void error(void);
void spib_fifo_init(void);
void spib_init(void);
//
// Globals
//
Uint16 LoopCount, flag;
Uint16 Ch_sel;
Uint16 x[8], xdot[8];
Uint16 xin[100];
Uint16 hh;
Uint16 kk = 0;
Uint16 xdot1, xdot2,xdot3,xdot4,xdot5;
float mult, mult1, x2, x1,x3,x4,x5,xd1,xd2,xd3,xd4,xd5, f1,f2,ids,iqs,idr,iqr,tem,tl,wr,j,l_lr,l_lr,l_ls,le,lm,lr,ls,rr,rs;

//
// Main
//

void main(void)
{
//    Uint16 sdata;  // send data
    Uint16 rdata;  // received data
    Uint16 i;
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example
    InitSpiaGpio();
    InitSpibGpio();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;
    // This is needed to write to EALLOW protected register
    PieVectTable.ADCINT1 = &adc_isr;
    // PieVectTable.ADCINT1 = &adc_isr;
    EDIS;
    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    // InitPeripherals(); // Not required for this example
    //
    InitAdc();  // For this example, init the ADC
    AdcOffsetSelfCal();

    //
    // Step 5. User specific code, enable interrupts:
    //

    //
    // Enable ADCINT1 in PIE
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    IER |= M_INT1;                     // Enable CPU Interrupt 1
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    LoopCount = 0;
    Ch_sel = 0;

    //
    // Configure ADC
    //
    EALLOW;

    spi_fifo_init();   // Initialize the SPI only
    spi_init();       // init SPI
    spib_fifo_init();   // Initialize the SPI only
    spib_init();
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode

    //
    // ADCINT1 trips after AdcResults latch
    //
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcRegs.INTSEL1N2.bit.INT1E = 1;  // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;  // Disable ADCINT1 Continuous mode

    //
    // setup EOC1 to trigger ADCINT1 to fire
    //
    AdcRegs.INTSEL1N2.bit.INT1SEL = 6;

    AdcRegs.ADCSOC0CTL.bit.CHSEL = 0;  // set SOC0 channel select to ADCINA0
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 1;  // set SOC1 channel select to ADCINA1
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 3;
    AdcRegs.ADCSOC4CTL.bit.CHSEL = 4;
    AdcRegs.ADCSOC5CTL.bit.CHSEL = 5;
    AdcRegs.ADCSOC6CTL.bit.CHSEL = 6;
    //
    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    //
    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL = 5;


    //
    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;

    //
    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;

    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC5CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC6CTL.bit.ACQPS = 6;

    // configure 1
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;

    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;

    EDIS;

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;        // Select SOC from CMPA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA = 0x0080;   // Set compare A value
    EPwm1Regs.TBPRD = 500;  // Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;        // count up and start

    //
    // Wait for ADC interrupt
    //
//    sdata = 0xFFF6;

    SpiaRegs.SPITXBUF = 0x8008;
    SpiaRegs.SPITXBUF = 0x900F;

    x[0] = 0x800;
    SpibRegs.SPITXBUF = 0xFF;
    ls = 0.0987;

    lr = 0.0987;

    le = 0.0065;
    l_ls = 0.0032;
    l_lr = 0.0032;
    lm = 0.0955;
    rs = 0.01;
    rr = 0.01;
    tl = 1;
    j = 1;

    while (1)
    {
        while (flag == 0)
            ;
        {
            flag = 0;
            x1 = -((float)x[1])*3.3/(0x0FFF) + 1.65;
            x2 = -((float)x[2])*3.3/(0x0FFF) + 1.65;
            x3 = -((float)x[3])*3.3/(0x0FFF) + 1.65;
            x4 = -((float)x[4])*3.3/(0x0FFF) + 1.65;
            x5 = -((float)x[5])*3.3/(0x0FFF) + 1.65;
            f1 = -((float)x[0])*3.3/(0x0FFF) + 1.65;
            f2 = -((float)x[6])*3.3/(0x0FFF) + 1.65;
            ids = 1 / le * (lr * x1 / lm - x3);
            iqs = 1 / le * (lr * x2 / lm - x4);
            idr = (1 / lm * (x1 - ids * l_ls)) - ids;
            iqr = (1 / lm * (x2 - iqs * l_ls)) - iqs;
            tem = 3 * lm*(iqs * idr - ids * iqr);
            wr = 2 * x5;

            GpioDataRegs.GPASET.bit.GPIO19 = 1;

            xd1 = f1 - rs * ids;
            xdot1 = (xd1 + 1.65) * 0x0FFF / 3.3;
            xdot1 = xdot1 & 0x0FFF;
            //xdot1 = x[0] & 0x0FFF;
            SpiaRegs.SPITXBUF = 0x0000 | xdot1;
            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

            for (hh = 0; hh < 80; hh++)
                ;
            ///////////////////////////////////////
            GpioDataRegs.GPASET.bit.GPIO19 = 1;

            xd2 = f2 - rs * iqs;
            xdot2 = (xd2 + 1.65) * 0x0FFF / 3.3;
            xdot2 = xdot2 & 0x0FFF;
            SpiaRegs.SPITXBUF = 0x1000 | xdot2;
            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
            for (hh = 0; hh < 80; hh++)
                ;
            /////////////////////////////////////////
            GpioDataRegs.GPASET.bit.GPIO19 = 1;
            xd3 = -wr * x4 - rr * idr;
            xdot3 = (xd3 + 1.65) * 0x0FFF / 3.3;
            xdot3 = xdot3 & 0x0FFF;
            SpiaRegs.SPITXBUF = 0x2000 | xdot3;
            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

            for (hh = 0; hh < 80; hh++)
                ;
            ////////////////////////////////////////
            GpioDataRegs.GPASET.bit.GPIO19 = 1;
            xd4 = wr * x3 - rr * iqr;
            xdot4 = (xd4 + 1.65) * 0x0FFF / 3.3;
            xdot4 = xdot4 & 0x0FFF;
            SpiaRegs.SPITXBUF = 0x3000 | xdot4;
            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

            for (hh = 0; hh < 80; hh++)
                ;
            ////////////////////////////////////
            GpioDataRegs.GPASET.bit.GPIO19 = 1;
            xd5 = (tem - tl) / j;
            xdot5 = (xd5 + 1.65) * 0x0FFF / 3.3;
            xdot5 = xdot5 & 0x0FFF;
            SpiaRegs.SPITXBUF = 0x4000 | xdot5;
            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

            for (hh = 0; hh < 80; hh++)
                ;

        }
//        if(flag == 1);
//               {
//               flag = 0;
//               GpioDataRegs.GPASET.bit.GPIO19 = 1;
//
//               xdot[0] = 0x07FF;
//       //                0xFFF - x[0];
//               SpiaRegs.SPITXBUF= 0x0000 | xdot[0];
//               GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
//               }
        //GpioDataRegs.GPBSET.bit.GPIO39 = 1;

//        while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){};
//        rdata = SpiaRegs.SPIRXBUF;
//
//        for(i=0;i<80;i++){}
//
//        GpioDataRegs.GPASET.bit.GPIO25 = 1;
//        xdot[1] = 0x07FF;
//        SpiaRegs.SPITXBUF= 0x1000 + xdot[1];
//        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
//
////        while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){};
////        rdata = SpiaRegs.SPIRXBUF;
//        for(i=0;i<80;i++){}
//        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;

    }

//    for(;;)
//    {
//        //
//        // Transmit data
//        //
//        if(flag == 1){
////            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
//            while(1){
//                sdata = 0x3000 + x[Ch_sel];
//                spi_xmit(sdata);
//                Ch_sel++;
//                if(Ch_sel == 8) {Ch_sel = 0; break;}
//            }
//            flag = 0;
////            for(i=0;1<10;i++){}
//            GpioDataRegs.GPASET.bit.GPIO19 = 1;
//
//        }
//
//        //
//        // Wait until data is received
//        //
////        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1)
////                            {
////
////                            }
//
//       // SpiaRegs.SPIFFTX.bit.TXFFST = 0;
//
//
//
//        //
//        //
//        // Check against sent data
//        //
//        rdata = SpiaRegs.SPIRXBUF;
////        if(rdata != sdata)
////        {
////            error();
////        }
//
////        sdata = 0x3FFF;
//        LoopCount++;
//    }
}

//
// adc_isr -
//
__interrupt void adc_isr(void)
{

    flag = 1;
    //
    // If 20 conversions have been logged, start over
    //
    x[0] = AdcResult.ADCRESULT0; //x[0] has -x
    x[1] = AdcResult.ADCRESULT1; //x[1] has sin(wt)
    x[2] = AdcResult.ADCRESULT2; //x[2] has -x2
    x[3] = AdcResult.ADCRESULT3;
    x[4] = AdcResult.ADCRESULT4;
    x[5] = AdcResult.ADCRESULT5;
    x[6] = AdcResult.ADCRESULT6;
    xin[kk] = x[0];
    kk = (kk + 1) % 100;
//    x[3] = AdcResult.ADCRESULT3;
//    x[4] = AdcResult.ADCRESULT4;
//    x[5] = AdcResult.ADCRESULT5;
//    x[6] = AdcResult.ADCRESULT6;
//    x[7] = AdcResult.ADCRESULT7;
//    if (x[0]<2048){
//        //led high
//        GpioDataRegs.GPASET.bit.GPIO1 = 1;
//    }
//    else
//    {
//        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
//    }

    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}

void error(void)
{
    __asm("     ESTOP0");
    // Test failed!! Stop!
    for (;;)
        ;
}

//
// spi_init -
//
void spib_init()
{
    SpibRegs.SPICCR.all = 0x0007;  // Reset on, rising edge, 16-bit char bits
    //
    // Enable master mode, normal phase, enable talk, and SPI int disabled.
    //
    SpibRegs.SPICTL.all = 0x0006;

    SpibRegs.SPIBRR = 0x000F;
    SpibRegs.SPICCR.all = 0x009F;   // Relinquish SPI from Reset
    SpibRegs.SPICCR.bit.SPICHAR = 7;

    SpibRegs.SPIPRI.bit.FREE = 1;  // Set so breakpoints don't disturb xmission

}

void

spi_init()
{
    SpiaRegs.SPICCR.all = 0x000F;  // Reset on, rising edge, 16-bit char bits
    //
    // Enable master mode, normal phase, enable talk, and SPI int disabled.
    //
    SpiaRegs.SPICTL.all = 0x0006;

    SpiaRegs.SPIBRR = 0x0002;
    SpiaRegs.SPICCR.all = 0x009F;   // Relinquish SPI from Reset
    SpiaRegs.SPICCR.bit.SPICHAR = 15;

    SpiaRegs.SPIPRI.bit.FREE = 1;  // Set so breakpoints don't disturb xmission

}

//
// spi_xmit -
//
void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF = a;
}

//
// spi_fifo_init -
//
void spi_fifo_init(void)
{
    //
    // Initialize SPI FIFO registers
    //
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;
}
void spib_fifo_init(void)
{
    //
    // Initialize SPI FIFO registers
    //
    SpibRegs.SPIFFTX.all = 0xE040;
    SpibRegs.SPIFFRX.all = 0x2044;
    SpibRegs.SPIFFCT.all = 0x0;
}

//
// End of File
//

