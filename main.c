
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
// variables for FFT----------------

//attemp1/2
double N_ADC_samples[N];
double FFTOutput[N/2];

//attempt 3
uint32_t real[N];
uint32_t imag[N];

//----------------------------------------------------------------------------------------------------------------
/*
//-------test main variables--------------------------------------------------------------------------------------------
#define TEST_LENGTH_SAMPLES 2048


extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES/2];


uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;


uint32_t refIndex = 213, testIndex = 0;
*/
//----------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------
//function prototypes---------------------------------------------------------------------------------------------
void initialisations();
void initClock();
void initADC();
void initSysTickClock();
void ADCIntHandler();
void SysTickIntHandler();
void debugLCD(char *s);
//double average;
void pwmInit();

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

/*float32_t average(float32_t *array) {
    float32_t output = 0;
    float32_t sum = 0;
    uint32_t size = sizeof(array) / sizeof(array[0]);
    uint32_t i;
    for (i = 0; i < size; i++) {
        sum +=array[i];
    }
    output = sum / size;
    return output;
}*/

//------------------------------------------------------------------------------------------------------------------------



void main(void) {

    //initialise peripherals and interrupts
    initialisations();
    IntMasterEnable();


    //Initialize variables
    char stringF[17];
    char stringG[17];
    //uint32_t fftSize = 256;
    //uint32_t doBitReverse = 1;
    //uint32_t ifftFlag = 0;
    //uint32_t average_frequency = 0;
    //float32_t avgFreq = 0.0;
    //uint32_t max;
    //uint32_t index;
    uint32_t i;
    uint32_t sum;
    double avg = 0.0;
    double doublesum;
    uint32_t input;
    uint32_t rgb[3] = {0};
 //   arm_rfft_fast_init_f32(&S, numSamples);





    while (1) {
        //background tasks

        //FFT computation Block---------------------------------------------------------------------------------
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





            // fft attempt 1---------------------------------------------------------------------

  //          arm_rfft_fast_f32(&S, N_ADC_samples, FFTOutput, ifftFlag);
            //arm_cfft_f32(&arm_cfft_sR_f32_len256, N_ADC_samples, ifftFlag, doBitReverse);
 //           arm_abs_f32(N_ADC_samples, FFTOutput, fftSize);
 //           arm_max_f32(FFTOutput, fftSize, &maxValue, &testIndex);
          //  avgFreq = average(FFTOutput);
          //  average_frequency= avgFreq;

/*
            //fft attempt 2----------------------------------------------------------------------
            arm_cfft_f32(&arm_cfft_sR_f32_len256, N_ADC_samples, ifftFlag, doBitReverse);

            // Process the data through the Complex Magnitude Module for
            //calculating the magnitude at each bin
            arm_cmplx_mag_f32(N_ADC_samples, FFTOutput, fftSize);

            // Calculates maxValue and returns corresponding BIN value
            arm_max_f32(FFTOutput, fftSize, &maxValue, &testIndex);
 */



            //fft attempt 3--------------------------------------------------------------------
            //init imag array to zero;
/*
            for( i=0; i<N; i++) imag[i] = 0;
           fix_fft(real, imag, log2N, 0);
           //get the power magnitude in each bin

           for ( i = 0; i < N/2; i++) {
               real[i] =ceil(sqrt((long)real[i] * (long)real[i] + (long)imag[i] * (long)imag[i]));
             }
           max = 0;
           index = 0;
           for (i = 1; i < N/2; i++) {
               if (real[i] > max && real[i] < 4000000 ) {
                   max = real[i];
                   index = i;
               }
           }

           uint32_t peak_freq = ((float32_t)index / (float32_t)N) * ADC_SAMPLE_RATE;


*/
            //debugging prints----------------------------------------------------------
/*            usnprintf(stringF, sizeof(stringF), "ADC samp: %4d", singleADCSample[0]);
            usnprintf(stringG, sizeof(stringG), "max-index: %4d", (uint32_t)avg);
            OLEDStringDraw("testing ADC", 0, 0);

            OLEDStringDraw(stringF, 0, 1);
            OLEDStringDraw("testing FFT", 0, 2);

            OLEDStringDraw(stringG, 0, 3);*/

            //--------------------------------------------------------------------------


            IntEnable(INT_ADC0SS0);

        //-----------------------------------------------------------------------------------------------------
        //SysCtlDelay(66666); //delay 1/100 sec


    }

}
}

    /*
int32_t main(void){
    initialisations();
  arm_status status;
  float32_t maxValue;

  status = ARM_MATH_SUCCESS;

 //  Process the data through the CFFT/CIFFT module

  arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, ifftFlag, doBitReverse);

  // Process the data through the Complex Magnitude Module for
  //calculating the magnitude at each bin
  arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);

  // Calculates maxValue and returns corresponding BIN value
  arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

  char s[17];
  OLEDStringDraw("testing FFT", 0, 0);
  usnprintf(s, sizeof(s), "FFT max: %3d", (uint32_t)maxValue);
  OLEDStringDraw(s, 0, 2);

  if (testIndex !=  refIndex)
  {
    status = ARM_MATH_TEST_FAILURE;
  }


  if ( status != ARM_MATH_SUCCESS)
  {
    while (1);
  }

  while (1);
}

*/


