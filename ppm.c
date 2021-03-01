/*
 * ppm.c
 *
 *  Created on: 2019年1月20日
 *      Author: wmy
 */

#include "ppm.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include <string.h>

////unit: us
//#define PPM_ENCODER_RESOLUTION 1
//unit: us
#define PPM_ENCODER_TOTAL_CH_VAL 20000
#define PPM_ENCODER_NEG_CH_VAL 500
//calculate arr value
#define CAL_ARR(ch_val)  (ch_val * ppm_period_us)
//gpio configure
#define PPM_GPIO_CLK   SYSCTL_PERIPH_GPIOB     //GPIOB 外设
#define PPM_GPIO_PORT  GPIO_PORTB_BASE        //GPIOB base
#define PPM_GPIO_PIN   GPIO_PIN_5             //pin5
//timer configure
#define PPM_TIMER_BASE    TIMER0_BASE          //timer0

typedef enum
{
    PPM_STATE_CH_NEG,
    PPM_STATE_CH_POS,
    PPM_STATE_IDLE_NEG,
    PPM_STATE_IDLE_POS,
    PPM_STATE_STOP,
}ppm_state_e;

//gpio cache
//static bool ppm_gpio_next_val;

//timer period per us, this will be caclulated when initialization
static uint32_t ppm_period_us = 1;

//ppm data buffer,  used to save user setting
#define PPM_DATA_BUF_NUM 2
static ppm_data_t ppm_data_buf[PPM_DATA_BUF_NUM];
static uint32_t ppm_data_buf_index;

//ppm data shadow buffer,  used to encode ppm
static ppm_data_t ppm_data_shadow;
static ppm_state_e ppm_shadow_state;
static uint32_t ppm_shadow_ch_idx;

static bool first_time = true;

static void ppm_gpio_init(void);
static void ppm_gpio_negative(void);
static void ppm_gpio_positive(void);
static void ppm_gpio_set_next(bool is_pos);
static void ppm_timer_config(void);
static void ppm_data_calculate_idle(ppm_data_t *ppm_data);
static void ppm_data_set_default(ppm_data_t *ppm_data);
static void ppm_data_buf_init(void);
static void ppm_data_shadow_init(void);
static void ppm_data_shadow_update(void);
static bool ppm_data_shadow_one_step(void);

void ppm_encoder_init(void)
{
    ppm_data_buf_init();
    ppm_data_shadow_init();  //将 结构体ppm_data_buf+i 的值复制到结构体ppm_data_shadow ，给 ppm_shadow_state初值PPM_STATE_CH_NEG
    ppm_gpio_init();  //使能GPIOB ,设置GPIOB pin5为输出 ，并置高pin5
    //delay_ms(500);
    ppm_timer_config();
}

static void ppm_gpio_init(void)
{
    ROM_SysCtlPeripheralEnable(PPM_GPIO_CLK);
    ROM_GPIOPinTypeGPIOOutput(PPM_GPIO_PORT, PPM_GPIO_PIN);
//    ppm_gpio_next_val = true;
    ppm_gpio_set_next(true);
}

static void ppm_gpio_negative(void)
{
    GPIOPinWrite(PPM_GPIO_PORT, PPM_GPIO_PIN, 0x00);
}

static void ppm_gpio_positive(void)
{
    GPIOPinWrite(PPM_GPIO_PORT, PPM_GPIO_PIN, 0xff);
}

static void ppm_gpio_set_next(bool is_pos)
{
//    if(ppm_gpio_next_val)
//    {
//        ppm_gpio_positive();
//    }
//    else
//    {
//        ppm_gpio_negative();
//    }
//    ppm_gpio_next_val = is_pos;

    if(is_pos)
    {
        ppm_gpio_positive();   //B5 置高
    }
    else
    {
        ppm_gpio_negative();   //B5置0
    }
}

static void ppm_timer_config(void)   //给ppm分配定时器模块/分配的是 timer0 的 timerA
{
    ppm_period_us = ROM_SysCtlClockGet() / 1000000;  //8

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Configure the two 32-bit periodic timers. //周期性计时
    //
    ROM_TimerConfigure(PPM_TIMER_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(PPM_ENCODER_TOTAL_CH_VAL));   //20000*8 =160 000

    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(PPM_TIMER_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    ROM_TimerEnable(PPM_TIMER_BASE, TIMER_A);
}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void Timer0AIntHandler(void)
{
    ROM_TimerIntClear(PPM_TIMER_BASE, TIMER_TIMA_TIMEOUT);

    if(first_time)
    {
        first_time = false;
        return;
    }

    if(ppm_data_shadow_one_step())
    {
        ppm_data_shadow_update();
    }
}


void ppm_encoder_set_data(ppm_data_t *ppm_data)
{
    memcpy(ppm_data_buf + ppm_data_buf_index, ppm_data, sizeof(ppm_data_t));
    ppm_data_calculate_idle(ppm_data_buf + ppm_data_buf_index);
    ppm_data_buf_index++;
    if(ppm_data_buf_index >= PPM_DATA_BUF_NUM)
    {
        ppm_data_buf_index = 0;
    }
}

static void ppm_data_calculate_idle(ppm_data_t *ppm_data)   //计算  空闲时间 idle_val 的
{
    uint32_t j;
    uint16_t idle_val = PPM_ENCODER_TOTAL_CH_VAL;  // 初始空闲值为20000
    for(j = 0; j < PPM_ENCODER_CHANNEL_NUM; j++)
    {
        idle_val -= PPM_ENCODER_NEG_CH_VAL;  // idle_val = idle_val - PPM_ENCODER_NEG_CH_VAL 》》 19500（18500），18000（17000） ， ，，
        idle_val -= ppm_data->ch_val[j];    //  第8次后 idle_val=8000
    }
    idle_val -= PPM_ENCODER_NEG_CH_VAL;   // 最终为为7500us
    ppm_data->idle_val = idle_val;
}


static void ppm_data_set_default(ppm_data_t *ppm_data)   //设ch_val初值（8个ppm波的高电平时间） ， 以及8个ppm完后的空闲时长
{
    uint32_t i;
    for(i = 0; i < PPM_ENCODER_CHANNEL_NUM; i++)
    {
        ppm_data->ch_val[i] = PPM_ENCODER_DEFFAULT_CH_VAL;  //将ch_val[8]中0~7 赋值1000（PPM_ENCODER_DEFFAULT_CH_VAL）
    }
    ppm_data_calculate_idle(ppm_data);
}

static void ppm_data_buf_init(void)
{
    uint32_t i;
    ppm_data_buf_index = 0;
    for(i = 0; i < PPM_DATA_BUF_NUM; i++)
    {
        ppm_data_set_default(ppm_data_buf + i);//传参时，ppm_data_buf[2] 其中0，1两元素都为结构体类型 ppm_data_t
    }
}

static void ppm_data_shadow_init(void)
{
    //ppm_data_set_default(&ppm_data_shadow);
    //ppm_shadow_state = PPM_STATE_CH_NEG;
    //ppm_shadow_ch_idx = 0;
    ppm_data_shadow_update();
}

static void ppm_data_shadow_update(void)
{
    uint32_t i;
    for(i = 0; i < PPM_DATA_BUF_NUM; i++)
    {
        if(i != ppm_data_buf_index)
        {
            memcpy(&ppm_data_shadow, ppm_data_buf + i, sizeof(ppm_data_t));
            break;
        }
    }                    //将ppm_data_buf + i 地址的内容拷到ppm_data_shadow

    if(i == PPM_DATA_BUF_NUM)
    {
        //impossible
        ppm_data_set_default(&ppm_data_shadow);
    }
    ppm_shadow_state = PPM_STATE_CH_NEG; //0
    ppm_shadow_ch_idx = 0;
}


static void handle_negative(void)
{
    //ppm_gpio_negative();
    ppm_gpio_set_next(false);
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(PPM_ENCODER_NEG_CH_VAL));  //500*1 = 0.5 ms
    ppm_shadow_state = PPM_STATE_CH_POS;

}

static void handle_positive(void)
{
    //ppm_gpio_positive();
    ppm_gpio_set_next(true);                    //ppm_shadow_ch_idx在初始化中ppm_data_shadow_update（）中赋值为0，所以
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(ppm_data_shadow.ch_val[ppm_shadow_ch_idx])); //这是取ppm_data_shadow.ch_val[ppm_shadow_ch_idx]也就是取0~7个通道的ch_val
    ppm_shadow_ch_idx++;  //改变了ppm_shadow_ch_idx的值，使其变成加1
    if(ppm_shadow_ch_idx >= PPM_ENCODER_CHANNEL_NUM)
    {
        ppm_shadow_state = PPM_STATE_IDLE_NEG;
    }
    else
    {
        ppm_shadow_state = PPM_STATE_CH_NEG;
    }

}

static void handle_idle_neg(void)
{
    //ppm_gpio_negative();
    ppm_gpio_set_next(false);
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(PPM_ENCODER_NEG_CH_VAL));
    ppm_shadow_state = PPM_STATE_IDLE_POS;
}

static void handle_idle_pos(void)
{
    //ppm_gpio_positive();
    ppm_gpio_set_next(true);
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(ppm_data_shadow.idle_val));
}


static bool ppm_data_shadow_one_step(void)
{
    bool should_update = false;

    switch(ppm_shadow_state)  //初始时 ppm_shadow_state 在 ppm_data_shadow_update（）中赋值为0
    {
        case PPM_STATE_CH_NEG:
            handle_negative();  //使B5低电平保持0.5ms
            break;
        case PPM_STATE_CH_POS:
            handle_positive();  // 执行ppm_shadow_ch_idx++, ppm_shadow_ch_idx从0开始加到7，使B5高电平保持了ppm_data_shadow.ch_val[ppm_shadow_ch_idx] us
            break;
        case PPM_STATE_IDLE_NEG:
            handle_idle_neg();  // 使B5低电平保持0.5ms
            break;
        case PPM_STATE_IDLE_POS:
            handle_idle_pos();  // 使B5高电平保持了ppm_data_shadow.idle_val us
            //ppm_shadow_state = PPM_STATE_STOP;
            should_update = true;
        default:
            //
            break;
    }

    return should_update;
}






