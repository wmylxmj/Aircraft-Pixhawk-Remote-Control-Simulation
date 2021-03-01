/*
 * ultrasonic.h
 *
 *  Created on: 2018Äê7ÔÂ13ÈÕ
 *      Author: wmy
 */

#ifndef ULTRASONIC_ULTRASONIC_H_
#define ULTRASONIC_ULTRASONIC_H_

extern bool ultrasonic_update_flag;
extern float ultrasonic_distance;
extern float ultrasonic_distance_history[10];

extern void UltrasonicConfigure(void);
extern void SonicTrig(void);

#endif /* ULTRASONIC_ULTRASONIC_H_ */
