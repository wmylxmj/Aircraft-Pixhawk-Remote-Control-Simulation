/*
 * receive.h
 *
 *  Created on: 2019Äê1ÔÂ20ÈÕ
 *      Author: wmy
 */

#ifndef RECEIVE_H_
#define RECEIVE_H_

extern char UART1_Rx_Buffers[50];
extern char UART1_Rx_Len;
extern char UART1_Sender_Address;
extern char UART1_Rx_Data[50];
extern bool UART1_Updated_Flag;

extern void Recive_UART1_Config(void);
extern void UART1IntHandler(void);



#endif /* RECEIVE_H_ */
