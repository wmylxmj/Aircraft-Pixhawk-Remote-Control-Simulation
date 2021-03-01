/*
 * filter.h
 *
 *  Created on: 2019Äê1ÔÂ25ÈÕ
 *      Author: wmy
 */

#ifndef FILTER_H_
#define FILTER_H_

#define len(a) sizeof(a)/sizeof(a[0])

//exponentially weighted average filter
typedef struct
{
    bool bias_correction;
    uint32_t t;
    double beta;
    double v_t;
    double v_t_prev;
    double theta_t;
    double output;
}EWA_Filter;

//moving average filter
typedef struct
{
    uint32_t window_size;
    double data_array[100];
    double average;
}MA_Filter;

extern void EWA_Filter_Init(EWA_Filter *ewa, double beta);
extern double EWA_Filter_Compute(EWA_Filter *ewa, double theta_t);

extern void MA_Filter_Init(MA_Filter *ma, uint32_t window_size);
extern double MA_Filter_Compute(MA_Filter *ma, double val);

#endif /* FILTER_H_ */
