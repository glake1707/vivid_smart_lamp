
/**
 * main.c
 * current purpose: use the ADC to sample the analog signal from the potentiometer and compute the FFT of the signal
 * and output the frequency of the pot signal on the OLED display.
 */
//Included libraries-----------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/uart.h"
#include "OrbitOLEDInterface.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include <math.h>

#include "arm_math.h"
#include "arm_const_structs.h"

//Macros-----------------------------------------------------------------------------------------------------------
#define ADC_SAMPLE_RATE 1024
#define N 16
#define log2N    8


//------------------------------------------------------------------------------------------------------------------
//Global Variables------------------------------------------------------------------------------------------------
static uint32_t singleADCSample[1];
static uint32_t sample_count = 0;
arm_rfft_fast_instance_f32 S;
uint32_t numSamples = 256;
uint32_t real[N];
uint32_t imag[N];



//----------------------------------------------------------------------------------------------------------------
//function prototypes---------------------------------------------------------------------------------------------
void initialisations();
void initClock();
void initADC();
void initSysTickClock();
void ADCIntHandler();
void SysTickIntHandler();
void debugLCD(char *s);
void pwmInit();
int fix_fft(short fr[], short fi[], short m, short inverse);
uint32_t linear(uint32_t freq);
void decode(int32_t val, int32_t *rgb);
void setDutyCycles(uint32_t *rgb);

//----------------------------------------------------------------------------------------------------------------
//helper functions------------------------------------------------------------------------------------------------

void initClock() {

    //initialize system clock--------------------------------------------------------------------
    // Set up the system clock (refer to pp.220 of the TM4C123 datasheet for an overview).
    // Many options exist here but basically the 16MHz crystal oscillator signal is
    // boosted to 400Mz via a phase-locked loop (PLL), divided by 2 and then again by 10
    // (via the SYSCTL_SYSDIV_10 setting) to make the system clock rate 20 MHz.
    SysCtlClockSet(SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_SYSDIV_10);

    SysCtlDelay(100);  // Allow time for the oscillator to settle down.
    // SysCtlDelay() is an API function which executes a 3-instruction loop the number of
    //   times specified by the argument).
}

void initADC() {
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    //CH9 corresponds to AIN9, CH0 corresponds to AIN0 etc...
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE |
                             ADC_CTL_END);
    //Ch1 = AIN1 = PE2 pin 7
    //CH9 = AIN9 = potentiometer

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

void initSysTickClock() {
    //initialize the clock for timed interrupts
    // Set up the period for the SysTick timer.  The SysTick timer period is
       // set as a function of the system clock. gives SysTick 'ADC_SAMPLE_RATE' interrupts per sec
       SysTickPeriodSet(SysCtlClockGet() / ADC_SAMPLE_RATE);
       //
       // Register the interrupt handler
       SysTickIntRegister(SysTickIntHandler);
       //
       // Enable interrupt and device
       SysTickIntEnable();
       SysTickEnable();
}

void SysTickIntHandler() {

    //trigger an ADC interrupt to generate one sample
    ADCProcessorTrigger(ADC0_BASE, 3);

    while(!ADCIntStatus(ADC0_BASE, 3, false));
}

void ADCIntHandler() {
    //uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h

    ADCSequenceDataGet(ADC0_BASE, 3, singleADCSample);
  //  float32_t data = (singleADCSample[0]);
  //  N_ADC_samples[sample_count] = data;
    if (sample_count >=N/2) {
        real[sample_count - N/2] = singleADCSample[0];
        if (real[sample_count - N/2] < 20)
        {
            real[sample_count - N/2] = 20;
        }
        real[sample_count - N/2] -= 20;
    }

    sample_count++;


    if (sample_count >= N*2 - N/2) {
        sample_count = 0;
        ADCIntClear(ADC0_BASE, 3);
        IntDisable(INT_ADC0SS0);
       // IntMasterDisable();
    }
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);


}


void initialisations() {
    initClock();
    initADC();
    initSysTickClock();
    pwmInit();
    OLEDInitialise();
}

void debugLCD(char *s) {
    OLEDStringDraw("testing ADC", 0, 0);
    usnprintf(s, sizeof(s), "ADC samp: %4d", singleADCSample[0]);
    OLEDStringDraw(s, 0, 2);
}

/**********************************************************************************************
 *                                END OF FUNCTION CODE                                        *
 **********************************************************************************************/






void main(void) {

    //initialise peripherals and interrupts
    initialisations();
    IntMasterEnable();


    //Initialize variables
    uint32_t i;
    uint32_t sum;
    double avg = 0.0;
    double doublesum;
    uint32_t input;
    uint32_t rgb[3] = {0};

    //background tasks
    while (1) {

        //RGB and PWM computation Block---------------------------------------------------------------------------------
        if (!IntIsEnabled(INT_ADC0SS0)) {
            //average real data array
            sum = 0;
            avg = 0;
            for (i = 0; i < N/2; i++) {
                sum += real[i];
            }
            doublesum = (double) sum;
            avg = doublesum / 8.0;
            input = ceil(avg / 1100.0 * 593.0 + 1.0);
            input = linear(input);
            decode(input, rgb);
            setDutyCycles(rgb);
        //-----------------------------------------------------------------------------------------------------

            IntEnable(INT_ADC0SS0);

        }

    }
}



