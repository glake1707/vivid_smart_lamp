
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
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include <math.h>

#include "arm_math.h"
#include "arm_const_structs.h"

//Macros-----------------------------------------------------------------------------------------------------------
#define ADC_SAMPLE_RATE 4096
#define N 256
#define log2N 8

#define min_freq 100
#define max_freq 2048
#define Red 99
#define RtoB 0.99
#define BtoG .32
#define GtoY .20
#define YtoW .36
#define dutyCycleStep 1
#define color_num 99/dutyCycleStep*6

//------------------------------------------------------------------------------------------------------------------
//Global Variables------------------------------------------------------------------------------------------------
static uint32_t singleADCSample[1];
static uint32_t sample_count = 0;

// variables for FFT----------------
short real[N];
short imag[N];

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

uint32_t linear(uint32_t freq)
{
    double scalar = (color_num - 1) / ((log10(max_freq) - log10(2)) - (log10(min_freq) - log10(2)));
    if (freq < min_freq) // Apply frequency floor
    {
        freq = min_freq;
    }
    else if (freq > max_freq) // Apply frequency ceiling
    {
        freq = max_freq;
    }

    double unscaled = log10(freq)-log10(2);
    uint32_t scaled = (uint32_t) ((unscaled - (log10(min_freq) - log10(2))) * scalar);
    return scaled;
}


void decode(uint32_t val, uint32_t *rgb)
{
    uint32_t maxVal = 99/dutyCycleStep*dutyCycleStep;
    uint32_t subRange = color_num / 6;
    if (val <= subRange) // Red to Purple
    {
        val = val;
        rgb[0] = Red;
        rgb[1] = dutyCycleStep;
        rgb[2] = ceil((double) val / subRange * 100 * RtoB);
    }
    else if (val <= 2 * subRange) // Purple to Blue
    {
        val = val - subRange - 1;
        rgb[0] = maxVal - (double) val / subRange * 99;
        rgb[1] = dutyCycleStep;
        rgb[2] = RtoB * 100;
    }
    else if (val <= 3 * subRange) // Blue to Teal
    {
        val = val - 2 * subRange;
        rgb[0] = dutyCycleStep;
        rgb[1] = ceil((double) val / subRange * 100 * BtoG);
        rgb[2] = 99;
    }
    else if (val <= 4 * subRange) // Teal to Green
    {
        val = val - 3 * subRange - 1;
        rgb[0] = dutyCycleStep;
        rgb[1] = BtoG * 100 - (double) val / subRange * 6;
        rgb[2] = maxVal - (double) val / subRange * 99;
    }
    else if (val <= 5 * subRange) // Green to Yellow
    {
        val = val - 4 * subRange;
        rgb[0] = ceil((double) val / subRange * 100 * RtoB);
        rgb[1] = GtoY * 100;
        rgb[2] = dutyCycleStep;
    }
    else // Yellow to White
    {
        val = val - 5 * subRange;
        rgb[0] = 99;
        rgb[1] = GtoY * 100;
        rgb[2] = ceil((double) val / subRange * 100 * YtoW);
    }
    rgb[0] = rgb[0] / dutyCycleStep * dutyCycleStep;
    rgb[1] = rgb[1] / dutyCycleStep * dutyCycleStep;
    rgb[2] = rgb[2] / dutyCycleStep * dutyCycleStep;
    if (rgb[0] == 0)
    {
        rgb[0] = dutyCycleStep;
    }
    if (rgb[1] == 0)
    {
        rgb[1] = dutyCycleStep;
    }
    if (rgb[2] == 0)
    {
        rgb[2] = dutyCycleStep;
    }
}


void setDutyCycles(uint32_t *rgb)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ceil((uint32_t)((double)rgb[2] / 100 * 30000))); //blue
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ceil((uint32_t)((double)rgb[1] / 100 * 30000))); //green
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ceil((uint32_t)((double)rgb[0] / 100 * 30000))); //red
}

void initialisations() {
    initClock();
    initADC();
    initSysTickClock();
    pwmInit();
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
    //char stringF[17];
    //char stringG[17];
    //uint32_t fftSize = 256;
    //uint32_t doBitReverse = 1;
    //uint32_t ifftFlag = 0;
    //uint32_t average_frequency = 0;
    //float32_t avgFreq = 0.0;
    uint32_t max;
    uint32_t index;
    uint32_t i;
    uint32_t peak_freq;
    long sum;
    double avg = 0.0;
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
            for (i = 0; i < N; i++) {
                sum += real[i];
            }
            avg = (double) sum / (double) N;

            //remove DC offset
            for (i = 0; i < N; i++) {
                real[i] -= avg;
            }

            //zero imaginary array
            for( i=0; i<N; i++){
                imag[i] = 0;
            }

            //perform fft
            fix_fft(real, imag, log2N, 0);

            //get the power magnitude in each bin
            for ( i = 0; i < N/2; i++) {
                real[i] = (short) sqrt(real[i] * real[i] + imag[i] * imag[i]);
            }
            max = 0;
            index = 0;
            for (i = 0; i < N/2; i++) {
                if (real[i] > max) {
                    max = real[i];
                    index = i;
                }
            }
            if (max < 10) {
                index = 1;
            }
            peak_freq = ((float32_t)index / (float32_t)N) * ADC_SAMPLE_RATE;

            input = linear(peak_freq);
            decode(input, rgb);
            setDutyCycles(rgb);

            // Restart
            IntEnable(INT_ADC0SS0);
        }
    }
}
