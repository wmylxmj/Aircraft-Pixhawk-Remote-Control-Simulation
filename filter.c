/*
 * filter.c
 *
 *  Created on: 2019Äê1ÔÂ25ÈÕ
 *      Author: wmy
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
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
#include "math.h"

#include "filter.h"

//exponentially weighted average filter
void EWA_Filter_Init(EWA_Filter *ewa, double beta)
{
    ewa->bias_correction = true;
    ewa->t = 0;
    ewa->beta = beta;
    ewa->v_t = 0;
    ewa->v_t_prev = 0;
    ewa->theta_t = 0;
    ewa->output = 0;
}

double EWA_Filter_Compute(EWA_Filter *ewa, double theta_t)
{
    ewa->t ++;
    ewa->theta_t = theta_t;
    ewa->v_t = ewa->beta*ewa->v_t_prev + (1-ewa->beta)*ewa->theta_t;
    ewa->v_t_prev = ewa->v_t;
    if(ewa->bias_correction)
    {
        ewa->output = ewa->v_t*1.0/(1.0-pow(ewa->beta, ewa->t));
    }
    else
    {
        ewa->output = ewa->v_t;
    }
    return ewa->output;
}

//moving average filter
void MA_Filter_Init(MA_Filter *ma, uint32_t window_size)
{
    uint32_t i = 0;
    ma->window_size = window_size;
    for(i=0;i<=ma->window_size-1;i++)
    {
        *(ma->data_array+i) = 0;
    }
    ma->average = 0;
}

double MA_Filter_Compute(MA_Filter *ma, double val)
{
    uint32_t i = 0;
    double average = 0;
    for(i=0;i<=ma->window_size-2;i++)
    {
        *(ma->data_array+i) = *(ma->data_array+i+1);
    }
    *(ma->data_array+ma->window_size-1) = val;
    for(i=0;i<=ma->window_size-1;i++)
    {
        average += *(ma->data_array+i);
    }
    average = average*1.0/ma->window_size;
    ma->average = average;
    return average;
}

