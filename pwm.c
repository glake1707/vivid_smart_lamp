
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


