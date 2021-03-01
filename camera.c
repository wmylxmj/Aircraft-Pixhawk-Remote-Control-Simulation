/*
 * camera.c
 *
 *  Created on: 2019Äê1ÔÂ26ÈÕ
 *      Author: czw
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "time.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "string.h"
#include "driverlib/timer.h"
#include "string.h"

#include "camera.h"


int x_hat=160, y_hat=120;
char UART3_Rx_Buffers[11];
bool UART3_Updated_Flag = false;

void Recive_UART3_Config(void)
{
     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
     GPIOPinConfigure(GPIO_PC6_U3RX);
     GPIOPinConfigure(GPIO_PC7_U3TX);
     GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6| GPIO_PIN_7);
     UARTConfigSetExpClk(UART3_BASE,SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
     IntEnable(INT_UART3);
     UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);
}

void UART3IntHandler(void)
{
    uint32_t ui32Status;
    uint8_t rx_buffer=0;
    static int count=0;
    static bool receive_flag=false;
    ui32Status = UARTIntStatus(UART3_BASE,true);
    UARTIntClear(UART3_BASE, ui32Status);
    while(UARTCharsAvail(UART3_BASE))
    {
        rx_buffer = (uint8_t)(UARTCharGetNonBlocking(UART3_BASE));
        if(rx_buffer=='a')
        {
            receive_flag = true;
            memset(UART3_Rx_Buffers,0,11);
            count =0;
        }
        if(rx_buffer=='z')
        {
            receive_flag = false;
            UART3_Updated_Flag =true;
        }
        if((receive_flag == true)&&(rx_buffer!='a'))
        {
            UART3_Rx_Buffers[count] = rx_buffer;
            count++;
        }
    }
}

void u3Data_handle(void)
{
    int i=0;
    int n0,n1;
    if(UART3_Updated_Flag==true)
    {
        x_hat = 0;
        y_hat = 0;
        //UARTprintf("%s\n",UART3_Rx_Buffers);
        n0=1,n1=1;
        UART3_Updated_Flag = false;
        for(i=7;i>0;i--)
        {
            if(UART3_Rx_Buffers[i-1] == ' ')
                UART3_Rx_Buffers[i-1] = '0';
        }
        for(i=3;i>0;i--)
        {
            x_hat += n0*(UART3_Rx_Buffers[i-1]-48);
            n0*=10;
        }
        for(i=7;i>4;i--)
        {
            y_hat += n1*(UART3_Rx_Buffers[i-1]-48);
            n1*=10;
        }
        //UARTprintf("x:%d,y:%d\n",x_hat,y_hat);
    }
}

void Camera_Hold_Black_Blob(void)
{
    UARTCharPutNonBlocking(UART3_BASE, 'w');
    UARTCharPutNonBlocking(UART3_BASE, 'm');
    UARTCharPutNonBlocking(UART3_BASE, 'y');
}

void Camera_Hold_Red_Blob(void)
{
    UARTCharPutNonBlocking(UART3_BASE, 'z');
    UARTCharPutNonBlocking(UART3_BASE, 'y');
    UARTCharPutNonBlocking(UART3_BASE, 'b');
}



