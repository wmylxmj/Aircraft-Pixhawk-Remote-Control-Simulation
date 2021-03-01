/*
 * ppm.h
 *
 *  Created on: 2019年1月20日
 *      Author: wmy
 */

#ifndef PPM_H_
#define PPM_H_

#include <stdint.h>

#define PPM_ENCODER_CHANNEL_NUM 8
#define PPM_ENCODER_DEFFAULT_CH_VAL 1000

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

typedef struct
{
    uint16_t ch_val[PPM_ENCODER_CHANNEL_NUM];
    //user needn't set
    uint16_t idle_val;
}ppm_data_t;

extern void ppm_encoder_init(void);
extern void ppm_encoder_set_data(ppm_data_t *ppm_data);


// ppm channal infos
/******************************************************************************************/

#define PPM_POWER_MIN 501
#define PPM_POWER_LOW 750
#define PPM_POWER_MIDDLE 1000
#define PPM_POWER_HIGH 1250
#define PPM_POWER_MAX 1500

#define PPM_ROLL_MIN 520
#define PPM_ROLL_LOW 765
#define PPM_ROLL_MIDDLE 1022
#define PPM_ROLL_HIGH 1218
#define PPM_ROLL_MAX 1424

#define PPM_YAW_MIN 500
#define PPM_YAW_LOW 751
#define PPM_YAW_MIDDLE 1008
#define PPM_YAW_HIGH 1250
#define PPM_YAW_MAX 1499

#define PPM_PITCH_MIN 501
#define PPM_PITCH_LOW 750
#define PPM_PITCH_MIDDLE 997
#define PPM_PITCH_HIGH 1250
#define PPM_PITCH_MAX 1496

#define PPM_ALTITUDE_HOLD_MODE 500
#define PPM_LAND_MODE 1500
#define PPM_STABILIZE_MODE 900

/******************************************************************************************/


//ppm channals
/******************************************************************************************/

#define PPM_ROLL_CHANNAL 0
#define PPM_PITCH_CHANNAL 1
#define PPM_POWER_CHANNAL 2
#define PPM_YAW_CHANNAL 3
#define PPM_STATION_CHANNAL 4

/******************************************************************************************/

#endif /* PPM_H_ */
