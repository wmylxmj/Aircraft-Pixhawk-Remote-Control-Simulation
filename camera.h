/*
 * camera.h
 *
 *  Created on: 2019Äê1ÔÂ26ÈÕ
 *      Author: czw
 */

#ifndef CAMERA_H_
#define CAMERA_H_

extern char UART3_Rx_Buffers[11];
extern int x_hat,y_hat;
extern bool UART3_Updated_Flag;


extern void Recive_UART3_Config(void);
extern void UART3IntHandler(void);
extern void u3Data_handle(void);

extern void Camera_Hold_Black_Blob(void);
extern void Camera_Hold_Red_Blob(void);

#endif /* CAMERA_H_ */
