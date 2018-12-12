
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"

//macros------------------------------------------------------------------------------------------
#define PWM_FREQUENCY 400
#define min_freq 10
#define max_freq 800
#define Red 99
#define RtoB 0.99
#define BtoG .32
#define GtoY .20
#define YtoW .36
#define dutyCycleStep 1
#define color_num 99/dutyCycleStep*6

//prototypes-----------------------------------------------------------------------------------------
void pwmInit(void);
uint32_t linear(uint32_t freq);
void decode(int32_t val, int32_t *rgb);
void setDutyCycles(uint32_t *rgb);

//------------------------------------------------------------------------------------------------
//global variables-----------------------------------------------------------------------------
uint32_t ui8Adjust = 440;
uint32_t ui32Period;

//---------------------------------------------------------------------------------------------

void pwmInit(void) {
  //  SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN| SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralReset (SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralReset (SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralReset (SYSCTL_PERIPH_GPIOE);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);



    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


    GPIOPinConfigure(GPIO_PB4_M0PWM2);

    GPIOPinConfigure(GPIO_PB5_M0PWM3);

    GPIOPinConfigure(GPIO_PE4_M0PWM4);



    GPIOPinTypePWM(GPIO_PORTB_BASE,(GPIO_PIN_4 | GPIO_PIN_5));

    GPIOPinTypePWM(GPIO_PORTE_BASE, (GPIO_PIN_4));



    //PWMGenConfigure(PWM0_BASE, PWM_GEN_0, (PWM_GEN_MODE_DOWN| PWM_GEN_MODE_NO_SYNC));

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, (PWM_GEN_MODE_DOWN| PWM_GEN_MODE_NO_SYNC));

    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, (PWM_GEN_MODE_DOWN| PWM_GEN_MODE_NO_SYNC));


    //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 30000);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 30000);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 30000);



    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 20000);     //duty cycle is 50%.    or add PWM_OUT_1 ?
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 20000);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 20000);     //duty cycle is 33%.    or add PWM_OUT_3 ?
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 20000);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 20000);       //duty cycle is 16%.    or add PWM_OUT_5 ?
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 20000);



    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT,   true);



    //PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, 10, 10);// here  enable too PWM_OUT_1 -invert  ?

    //PWMDeadBandEnable(PWM0_BASE, PWM_GEN_1, 10, 10); // here enable too  PWM_OUT_3 -invert ?

    //PWMDeadBandEnable(PWM0_BASE, PWM_GEN_2, 10, 10);// here enable too  PWM_OUT_5 -invert  ?

     //value of 10 means that Deadband is (1/40)*10 us?  I have PWMclock - 40Mhz



    //PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
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


void decode(int32_t val, int32_t *rgb)
{
    int32_t maxVal = 99/dutyCycleStep*dutyCycleStep;
    int32_t subRange = color_num / 6;
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













void pwmInit1(void) {

/*PWM frequency in Hz*/
    SysCtlPeripheralReset (SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralReset (SYSCTL_PERIPH_GPIOB);


    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);//divide the clock by 64


/*
    Configure PB7 as M0PWM1
    Configure PB4 as M0PWM2
    Configure PB5 as M0PWM3 page651
  */

  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  /*Using a specific pin on the PWM module*/
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
  GPIOPinConfigure(GPIO_PB7_M0PWM1);

  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
  GPIOPinConfigure(GPIO_PB4_M0PWM2);

  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
  GPIOPinConfigure(GPIO_PB5_M0PWM3);
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7|GPIO_PIN_4|GPIO_PIN_5);

  /*PWM clock and frequency*/
 // ui32PWMClock = SysCtlClockGet() / 64;
 // ui32Period = SysCtlClockGet() / PWM_FREQUENCY / SYSCTL_PWMDIV_64;

  /*Configure the PWM to count up/down without synchronisation*/
  PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

  /*SYNC configure*/
  PWMSyncTimeBase(PWM1_BASE, PWM_GEN_0_BIT|PWM_GEN_1_BIT);
  PWMSyncUpdate(PWM1_BASE, PWM_GEN_0_BIT|PWM_GEN_1_BIT);


  /*Setting the period of the PWM output*/
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 1666);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 1666);

  /*Output can be toggled on & off*/

  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 1600 / 2);
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, 500 / 2);
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 700 / 2);
  //SysCtlDelay(10000); // SysCtlDelay adds a delay in clock ticks to the loop.

  /*PWM Dead Band Initialization*/
  PWMDeadBandEnable(PWM1_BASE,PWM_GEN_0,16,16);
  PWMDeadBandEnable(PWM1_BASE,PWM_GEN_1,16,16);

  PWMGenIntTrigEnable(PWM0_BASE,PWM_GEN_0,PWM_TR_CNT_ZERO);


  /*Enables the PWM module to start modify the pins*/
  PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT|PWM_OUT_4_BIT|PWM_OUT_5_BIT, false);
  //PWMOutputState(PWM1_BASE, PWM_OUT_4_BIT, true);
  //PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);

  /*Enables the PWM generator*/
   PWMGenEnable(PWM1_BASE, PWM_GEN_0);
   PWMGenEnable(PWM1_BASE, PWM_GEN_1);



}
//LED test run in the main
/*
while(1) // LED Test
{
    uint32_t rgb[3] = {0};
    uint32_t i;
    uint32_t val;
    for (i = 100; i <= 10000; i=i*1.01)
    {
        val = linear(i);
        decode(val, rgb);
        setDutyCycles(rgb);
        SysCtlDelay(66666);
    }

}
} */


