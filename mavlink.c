/*
 * mavlink.c
 *
 *  Created on: 2019年1月20日
 *      Author: wmy
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include <string.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom_map.h"

#include "mavlink.h"

#define PAYLOAD_BUF_NUM 2          //姿态信息的数组的维度

uint8_t rx_buffer;  //缓存Mavlink接收串口的数据
bool newAttiFlag = false;  //姿态更新标志
bool newHeightFlag = false;//高度更新标志

//缓存数组
uint8_t Rx_Buffer_attitude[40] = {0};
uint8_t start_receive_attitude = 0;
uint8_t Rx_Buffer_height[20]= {0};
uint8_t start_receive_height =0;

int int_pitch, int_roll, int_yaw, int_mavlink_height;
float pitch, roll, yaw , mavlink_height;

uint8_t payload_buf_index ; //姿态信息的数组的索引值

Attitude_Payload Source_attitude_payload[PAYLOAD_BUF_NUM] ;
Rangefinder_Payload Source_Rangefinder_payload ;

void Mavlink_DateInit(void);

void Mav_recive_UART2_Config(void)
{
     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
     // Enable pin PD7 for UART2 U2TX
     // First open the lock and select the bits we want to modify in the GPIO commit register.
     HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
     HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
     ////此前最好先使能处理器中断IntMasterEnable();///
     GPIOPinConfigure(GPIO_PD6_U2RX);
     GPIOPinConfigure(GPIO_PD7_U2TX);
     GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
     UARTConfigSetExpClk(UART2_BASE,SysCtlClockGet(), 57600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
     UARTFIFOEnable(UART2_BASE);
     UARTFIFOLevelSet(UART2_BASE,UART_FIFO_TX4_8,UART_FIFO_RX2_8); //默认的发送FIFO深度为1/2，设置接受FIFO为1/4
     //使能UART2 中断
     IntEnable(INT_UART2);
     UARTIntEnable(UART2_BASE, UART_INT_RX | UART_INT_RT);
}

void Attitude_Data_Store (void)
{
    payload_buf_index++;
    if(payload_buf_index >= PAYLOAD_BUF_NUM)
    {
       payload_buf_index = 0;
    }
    memcpy(&(Source_attitude_payload[payload_buf_index]),Rx_Buffer_attitude+10,sizeof(Attitude_Payload));
    newAttiFlag = true; //用于提示MCU,姿态信息已更新
}

void UART2IntHandler(void)
{
    uint32_t ui32Status;
    static uint8_t index_i=0,index_j=0;
    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART2_BASE, true);
    //UARTIntStatus()
    // Clear the asserted interrupts.
    UARTIntClear(UART2_BASE, ui32Status);
    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART2_BASE))
    {
        // Read the next character from the UART and write it back to the UART.
        //UARTCharPutNonBlocking(UART0_BASE,UARTCharGetNonBlocking(UART0_BASE));
        rx_buffer =(uint8_t)(UARTCharGetNonBlocking(UART2_BASE));
        if(rx_buffer == MAVLINK_STX)  //根据帧头STX来筛选
        {
            start_receive_height = 1;
            start_receive_attitude = 1;
        }
        if(index_i>=2)  //再根据LEN来筛选
        {
            if(Rx_Buffer_height[1] == MSG_RANGEFINDER_LEN)
            {
                start_receive_attitude = 0;//0 @wb 2018-7-12
                index_j=0;
            }
            else if(Rx_Buffer_attitude[1] == MSG_ATTITUDE_PAYLOAD_LEN)
            {
                start_receive_height = 0;//0 @wb 2018-7-12
                index_i=0 ;
            }
            else
            {
                start_receive_height = 0;
                start_receive_attitude = 0;
                index_i=0,index_j=0;
            }
        }
        if(start_receive_height == 1)  //开始解析高度数据
        {
            Rx_Buffer_height[index_i++]=rx_buffer;
            if(Rx_Buffer_height[1] == MSG_RANGEFINDER_LEN && Rx_Buffer_height[5] == MSG_ID_RANGEFINDER && Rx_Buffer_height[15] != 0)
            {
               memcpy(&Source_Rangefinder_payload,Rx_Buffer_height+6,sizeof(Rangefinder_Payload));
               newHeightFlag = true; //用于提示MCU,高度信息已更新
            }
            if( index_i == MSG_RANGEFINDER_WHOLEN)  //如果Rx_Buffer_height里接收满了16个字节，则清零它
            {
                   index_i = 0;
                   start_receive_height = 0;
                   memset(Rx_Buffer_height,0,20); //将Rx_Buffer_height里的内容清零
             }
         }
         if(start_receive_attitude == 1) //开始解析姿态数据
         {
            Rx_Buffer_attitude[index_j++] = rx_buffer;
            if(Rx_Buffer_attitude[1] == MSG_ATTITUDE_PAYLOAD_LEN && Rx_Buffer_attitude[5] == MSG_ATTITUDE_ID && Rx_Buffer_attitude[35] != 0)
            {
                //memcpy(&source_attitude_payload,Rx_Buffer_attitude+10,sizeof(Msg_Attitude_Payload));
                //calculate_angle_test();
                Attitude_Data_Store();
            }
            if(index_j == MSG_ATTITUDE_WHOLEN)  //收满36个字节后,表明收完一个#30的信息包。清零Rx_Buffer_angle
            {
                index_j = 0;
                start_receive_attitude =0;
                memset(Rx_Buffer_attitude,0,40);
             }
         }
         /* End user code. Do not edit comment generated here */
    }
}

void Attitude_init (void)
{
    memset(&Source_attitude_payload, 0, sizeof(Attitude_Payload) * 2);
}

void Distance_init (void)
{
    memset(&Source_Rangefinder_payload, 0, sizeof(Rangefinder_Payload));
}

void Mavlink_DateInit(void)
{
    Attitude_init();
    Distance_init();
    Mav_recive_UART2_Config();
}

//获得mavlink数据
void MavLink_Data_Get(void)
{
    if(newAttiFlag)
    {
        newAttiFlag = false;
        pitch =(Source_attitude_payload[payload_buf_index].pitch)*57.325; //度
        roll = (Source_attitude_payload[payload_buf_index].roll)*57.325;
        yaw = (Source_attitude_payload[payload_buf_index].yaw)*57.325; //-180 ~180
        if(Source_attitude_payload[payload_buf_index].yaw >= 0)       //0~360
        {
            yaw = (Source_attitude_payload[payload_buf_index].yaw)*57.325;
        }
        else
        {
            yaw = (Source_attitude_payload[payload_buf_index].yaw)*57.325 + 360;
        }
        int_pitch =(int)(pitch);
        //UARTprintf("\n int_pitch= %d", int_pitch);
        int_roll = (int)(roll);
        //UARTprintf("\n int_roll= %d", int_roll);
        int_yaw = (int)(yaw);
        //UARTprintf("\n int_yaw= %d",int_yaw);
    }
    if(newHeightFlag)
    {
        newHeightFlag = false;
        mavlink_height = (Source_Rangefinder_payload.distance)*1000;  //mm
        int_mavlink_height = (int)(mavlink_height);
        //UARTprintf("\n int_distance= %d", int_distance);
    }
}

