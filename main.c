
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
#define ADC_SAMPLE_RATE 16000
#define N 256


//------------------------------------------------------------------------------------------------------------------
//Global Variables------------------------------------------------------------------------------------------------
static uint32_t singleADCSample[1];
static float32_t N_ADC_samples[N];
static float32_t FFTOutput[N/2];
static uint32_t sample_count = 0;


//----------------------------------------------------------------------------------------------------------------
//function prototypes---------------------------------------------------------------------------------------------
void initialisations();
void initClock();
void initADC();
void initSysTickClock();
void ADCIntHandler();
void SysTickIntHandler();
void debugLCD(char *s);
float32_t average(float32_t *array);

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
    //do more things?
}

void ADCIntHandler() {
    //uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h

    ADCSequenceDataGet(ADC0_BASE, 3, singleADCSample);
    N_ADC_samples[sample_count++] = (float32_t)(*singleADCSample);
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);

    if (sample_count == (N-1)) {
        sample_count = 0;
       // IntDisable(INT_ADC0SS0);
        IntMasterDisable();
    }


    // Place it in the circular buffer (advancing write index)
   // writeCircBuf (&g_inBuffer, ulValue);


}


void initialisations() {
    initClock();
    initADC();
    initSysTickClock();
}

void debugLCD(char *s) {
    OLEDStringDraw("testing ADC", 0, 0);
    usnprintf(s, sizeof(s), "ADC samp: %4d", singleADCSample[0]);
    OLEDStringDraw(s, 0, 2);
}

float32_t average(float32_t *array) {
    float32_t output = 0;
    float32_t sum = 0;
    uint32_t size = sizeof(array) / sizeof(array[0]);
    uint32_t i;
    for (i = 0; i < size; i++) {
        sum +=array[i];
    }
    output = sum / size;
    return output;
}

//------------------------------------------------------------------------------------------------------------------------



void main(void) {

    //initialise peripherals and interrupts
    initialisations();
    IntMasterEnable();
    OLEDInitialise();

    //initialise variables
    char stringF[17];
    uint32_t fftSize = 256;
    uint32_t doBitReverse = 1;
    uint32_t ifftFlag = 0;
    uint32_t average_frequency = 0;
    float32_t avgFreq = 0.0;
    float32_t maxValue;
    uint32_t testIndex;


    while (1) {
        //background tasks

        //FFT computation Block---------------------------------------------------------------------------------
        if (sample_count == 0) {

            arm_cfft_f32(&arm_cfft_sR_f32_len256, N_ADC_samples, ifftFlag, doBitReverse);
            arm_cmplx_mag_f32(N_ADC_samples, FFTOutput, fftSize);
            //arm_max_f32(FFTOutput, fftSize, &maxValue, &testIndex);
            avgFreq = average(FFTOutput);
            average_frequency = avgFreq;

            //debugging prints----------------------------------------------------------
            OLEDStringDraw("testing ADC", 0, 0);
            usnprintf(stringF, sizeof(stringF), "ADC samp: %4d", singleADCSample[0]);
            OLEDStringDraw(stringF, 0, 1);
            OLEDStringDraw("testing FFT", 0, 2);
            usnprintf(stringF, sizeof(stringF), "Avg Freq: %4d", average_frequency);
            OLEDStringDraw(stringF, 0, 3);

            //--------------------------------------------------------------------------
            IntMasterEnable();
        }
        //-----------------------------------------------------------------------------------------------------


        SysCtlDelay(66666); //delay 1/100 sec
    }

}

