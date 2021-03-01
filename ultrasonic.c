/*
 * ultrasonic.c
 *
 *  Created on: 2018Äê7ÔÂ13ÈÕ
 *      Author: wmy
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "ultrasonic.h"

#define len(a) sizeof(a)/sizeof(a[0])

bool ultrasonic_update_flag = false;
float ultrasonic_distance = 0;
float ultrasonic_distance_history[10] = {0};

void GPIOE3_Init(void)
{
    // Enable the GPIO port that is used for the trig.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    // Enable the GPIO pin for the trig(PF1).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Output low.
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x0);
}

void SonicTrig(void)
{
    // Output high.
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
    // Delay some times, 15us.
    ROM_SysCtlDelay(15*(SysCtlClockGet()/3000000));
    // Output low.
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x0);
}

// Interrupt handler of wtimer0B.
void WTimer0BIntHandler(void)
{
    uint32_t i=0;
    static uint16_t ui16EdgeCount = 0;  // Indicates the edge count.
    static uint32_t ui32Time[2]={0};    // Save the time value.
    double fPeriod = 0;
    // Clear the TIMER_CAPB_EVENT interrupt status.
    TimerIntClear(WTIMER0_BASE, TimerIntStatus(WTIMER0_BASE, true));
    // Read the timer value.    ____|-----|_____|-----|____
    ui32Time[ui16EdgeCount++] = TimerValueGet(WTIMER0_BASE,TIMER_B);
    if(ui16EdgeCount > 1)
    {
        // Reset the count.
        ui16EdgeCount = 0;
        //  Calculate the period time.
        fPeriod = ui32Time[1] > ui32Time[0] ? ui32Time[1] - ui32Time[0] : ui32Time[1] - ui32Time[0] + 0xFFFFFFFF;
        // Calculate the distance.
        // ui32Distance = (fPeriod * 12.5ns * V)/2;
        ultrasonic_distance = fPeriod*0.0021875;
        //10-1=9
        for(i=len(ultrasonic_distance_history)-1;i>=1;i--)
        {
            //a[9]=a[8] a[1]=a[0]
            *(ultrasonic_distance_history+i) = *(ultrasonic_distance_history+i-1);
        }
        *(ultrasonic_distance_history+0) = ultrasonic_distance;
        // Set the flag indicates has finish calculate.
        ultrasonic_update_flag=true;
    }
}

void WTimer0BConfigure(void)
{
    // Enable the peripheral GPIOC.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    // Enable the peripheral wtimer0.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    // Configures the alternate function of a GPIO pin.
    // This configure the GPIOC5 works in the input-capture mode.
    GPIOPinConfigure(GPIO_PC5_WT0CCP1);
    // Configures GPIOC5 for use by the Timer peripheral.
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5);
    // Configure wtimer0: Two half-width timers, and Timer B as event up-count timer.
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_B_CAP_TIME_UP | TIMER_CFG_SPLIT_PAIR);
    // Controls the wtimer0B event type: Both positive and negative edges.
    TimerControlEvent(WTIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);
    // Register the wtimer0B interrupt handler dynamic.
    TimerIntRegister(WTIMER0_BASE, TIMER_B, WTimer0BIntHandler);
    // Enable the wtimer0B capture-event interrupt.
    TimerIntEnable(WTIMER0_BASE, TIMER_CAPB_EVENT);
    // Enable the wtimer0B interrupt.
    IntEnable(INT_WTIMER0B);
    // Set the interrupt priority, 0xE0-0x0.
    IntPrioritySet(INT_WTIMER0B, 0x0);
    // Enable the wtimer0B.
    TimerEnable(WTIMER0_BASE, TIMER_B);
}

void UltrasonicConfigure(void)
{
    GPIOE3_Init();
    WTimer0BConfigure();
}
