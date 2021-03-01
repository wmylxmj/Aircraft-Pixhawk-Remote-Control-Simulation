/*
 * main.c
 *
 *  Created on: 2019年1月25日
 *      Author: wmy
 */

#include <stdint.h>
#include <stdio.h>
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

#include "ultrasonic.h"
#include "receive.h"
#include "mavlink.h"
#include "ppm.h"
#include "filter.h"
#include "camera.h"

#define CALIBRATION_MODE false //电调校准模式
#define AIRCRAFT_MISSION_NUM 7

//高度控制结构体
typedef struct
{
    int altitude_source; //超声波来源 0.mavlink 1.自带超声波
    bool altitude_hold_flag; //高度控制标志位
    uint32_t target_altitude; //目标高度mm
    uint32_t altitude_hold_fault_tolerant_lower_limit; //容错范围 mm
    uint32_t altitude_hold_fault_tolerant_upper_limit; //容错范围 mm
    uint32_t power_increment_max; //油门最大增量 ms
    uint32_t power_decrease_max; //油门最大减小量 ms
    uint32_t original_height; //初始地面高度 mm
    uint32_t overshoot_amount_threshold; //最大超调量(达到容错上限加此高度时,油门减小max) mm
    uint32_t dead_zone_up_section; //死区区间 ms
    uint32_t dead_zone_down_section; //死区区间 ms
}AltitudeControler;

//定点控制结构体
typedef struct
{
    bool position_hold_flag; //定点标志位
    bool x_reverse; //x翻转
    bool y_reverse; //y翻转
    int x0; //当前坐标(屏幕中心点X坐标 固定值)
    int y0; //当前坐标(屏幕中心点Y坐标 固定值)
    int x_hat; //预期坐标(目标点在屏幕中X坐标)
    int y_hat; //预期坐标(目标点在屏幕中Y坐标)
    int x_relative; //相对坐标X(x0-x_hat)
    int y_relative; //相对坐标Y(y0-y_hat)
    int x_error; //误差项
    int y_error; //误差项
    int x_last_error; //上次误差
    int y_last_error; //上次误差
    int x_integration; //积分项
    int y_integration; //积分项
    double x_kp; //kp
    double x_ki; //ki
    double x_kd; //kd
    double y_kp; //kp
    double y_ki; //ki
    double y_kd; //kd
}PositionControler;

//ppm
ppm_data_t ppm_data;

AltitudeControler altitude_controler;
PositionControler position_controler;

//高度滤波器
EWA_Filter altitude_ewa;

int aircraft_mission = 0;
bool aircraft_start_flag = false; //启动标志位

void UART1_Data_Pros(void);
int AltitudeGet(void);

//高度控制初始化
void AltitudeControlerInit(void)
{
    altitude_controler.altitude_source=0;//from mavlink
    altitude_controler.altitude_hold_flag=false;
    altitude_controler.target_altitude=1000;
    altitude_controler.altitude_hold_fault_tolerant_upper_limit=50;
    altitude_controler.altitude_hold_fault_tolerant_lower_limit=75;
    altitude_controler.power_increment_max=35;
    altitude_controler.power_decrease_max=35;
    altitude_controler.original_height=49;
    altitude_controler.overshoot_amount_threshold=380;
    altitude_controler.dead_zone_up_section=100;
    altitude_controler.dead_zone_down_section=100;
    //滤波器初始化
    EWA_Filter_Init(&altitude_ewa, 0.25);
}

//定点控制初始化
void PositionControlerInit(void)
{
    position_controler.position_hold_flag=false;
    position_controler.x_reverse=false;
    position_controler.y_reverse=false;
    position_controler.x0=160;
    position_controler.y0=120;
    position_controler.x_hat=160;
    position_controler.y_hat=120;
    position_controler.x_relative=0;
    position_controler.y_relative=0;
    position_controler.x_error=0;
    position_controler.y_error=0;
    position_controler.x_last_error=0;
    position_controler.y_last_error=0;
    position_controler.x_integration=0;
    position_controler.y_integration=0;
    position_controler.x_kp=0.435;
    position_controler.x_ki=0;
    position_controler.x_kd=1.11;
    position_controler.y_kp=0.435;
    position_controler.y_ki=0;
    position_controler.y_kd=1.11;
}

//高度控制
void Altitude_Control(void)
{
    uint32_t error_altitude_abs=0;//误差的绝对值
    double kp=0;//比例系数
    uint32_t power_increment_abs=0;//油门改变量绝对值
    uint32_t power_output=0; //油门输出值
    double descent_rate=0;//下调率
    if(altitude_controler.altitude_hold_flag)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3));
        //当前高度小于目标高度
        if(AltitudeGet()<altitude_controler.target_altitude-\
                altitude_controler.altitude_hold_fault_tolerant_lower_limit)
        {
            error_altitude_abs=(altitude_controler.target_altitude-\
                    altitude_controler.altitude_hold_fault_tolerant_lower_limit)-AltitudeGet();
            kp=(altitude_controler.power_increment_max*1.0/((altitude_controler.target_altitude-\
                    altitude_controler.altitude_hold_fault_tolerant_lower_limit)-altitude_controler.original_height));
            power_increment_abs=(int)(kp*error_altitude_abs);
            //死区向上油门增量
            power_output=PPM_POWER_MIDDLE+altitude_controler.dead_zone_up_section+power_increment_abs;
            if(power_output>PPM_POWER_MIDDLE+altitude_controler.dead_zone_up_section+\
                    altitude_controler.power_increment_max)
            {
                power_output=PPM_POWER_MIDDLE+altitude_controler.dead_zone_up_section+\
                        altitude_controler.power_increment_max;
            }
            ppm_data.ch_val[PPM_POWER_CHANNAL]=(int)power_output;
            ppm_encoder_set_data(&ppm_data);
        }
        else if(AltitudeGet()>=altitude_controler.target_altitude-\
                altitude_controler.altitude_hold_fault_tolerant_lower_limit&&\
                AltitudeGet()<=altitude_controler.target_altitude+\
                altitude_controler.altitude_hold_fault_tolerant_upper_limit)
        {
            //50%
            power_output=PPM_POWER_MIDDLE;
            ppm_data.ch_val[PPM_POWER_CHANNAL]=(int)power_output;
            ppm_encoder_set_data(&ppm_data);
        }
        else if(AltitudeGet()>altitude_controler.target_altitude+\
                altitude_controler.altitude_hold_fault_tolerant_upper_limit)
        {
            //误差高度
            error_altitude_abs=AltitudeGet()-(altitude_controler.target_altitude+\
                    altitude_controler.altitude_hold_fault_tolerant_upper_limit);
            descent_rate=(altitude_controler.power_decrease_max*1.0/(altitude_controler.overshoot_amount_threshold-\
                    altitude_controler.altitude_hold_fault_tolerant_upper_limit));
            power_increment_abs=(int)(descent_rate*error_altitude_abs);
            //死区下调
            power_output=PPM_POWER_MIDDLE-altitude_controler.dead_zone_down_section-power_increment_abs;
            if(power_output<PPM_POWER_MIDDLE-altitude_controler.dead_zone_down_section-\
                    altitude_controler.power_decrease_max)
            {
                power_output=PPM_POWER_MIDDLE-altitude_controler.dead_zone_down_section-\
                        altitude_controler.power_decrease_max;
            }
            ppm_data.ch_val[PPM_POWER_CHANNAL]=(int)power_output;
            ppm_encoder_set_data(&ppm_data);
        }
    }
}

//定点控制
void Position_Control(void)
{
    double  x_pid, y_pid; //pid输出量
    uint32_t x_out, y_out; //最终输出量
    if(position_controler.position_hold_flag==true)
    {
        if(AltitudeGet()>350)//35cm
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2));
            position_controler.x_hat = x_hat;
            position_controler.x_relative=position_controler.x0-position_controler.x_hat;
            position_controler.x_last_error = position_controler.x_error;
            position_controler.x_error=0-position_controler.x_relative;
            position_controler.x_integration += position_controler.x_error;
            x_pid = position_controler.x_kp*position_controler.x_error + \
                    position_controler.x_ki*position_controler.x_integration + \
                    position_controler.x_kd*(position_controler.x_error-position_controler.x_last_error);
            if(position_controler.x_reverse==false)
            {
                x_out = (int)(PPM_ROLL_MIDDLE + x_pid);
            }
            else
            {
                x_out = (int)(PPM_ROLL_MIDDLE - x_pid);
            }
            position_controler.y_hat = y_hat;
            position_controler.y_relative=position_controler.y0-position_controler.y_hat;
            position_controler.y_last_error = position_controler.y_error;
            position_controler.y_error=0-position_controler.y_relative;
            position_controler.y_integration += position_controler.y_error;
            y_pid = position_controler.y_kp*position_controler.y_error + \
                    position_controler.y_ki*position_controler.y_integration + \
                    position_controler.y_kd*(position_controler.y_error-position_controler.y_last_error);
            if(position_controler.y_reverse==false)
            {
                y_out = (int)(PPM_PITCH_MIDDLE + y_pid);
            }
            else
            {
                y_out = (int)(PPM_PITCH_MIDDLE - y_pid);
            }
            ppm_data.ch_val[PPM_ROLL_CHANNAL]=x_out;
            ppm_data.ch_val[PPM_PITCH_CHANNAL]=y_out;
            ppm_encoder_set_data(&ppm_data);
        }
        else
        {
            if(aircraft_mission==2)
            {
                ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE-15;
                ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
                ppm_encoder_set_data(&ppm_data);
            }
            else
            {
                ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
                ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
                ppm_encoder_set_data(&ppm_data);
            }
        }
    }
    else
    {
        ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
        ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
        ppm_encoder_set_data(&ppm_data);
    }
}

//高度获取
int AltitudeGet(void)
{
    if(altitude_controler.altitude_source==0)
    {
        return (int)altitude_ewa.output;
    }
    else if(altitude_controler.altitude_source==1)
    {
        return (int)ultrasonic_distance;
    }
    return 0;
}

//数据接收
void AircraftDataReceive(void)
{
    //姿态数据
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
    //高度数据
    if(altitude_controler.altitude_source==0)
    {
        if(newHeightFlag)
        {
            newHeightFlag = false;
            mavlink_height = (Source_Rangefinder_payload.distance)*1000;  //mm
            int_mavlink_height = (int)(mavlink_height);
            if(int_mavlink_height>=30 && int_mavlink_height<2500)
            {
                //滤波
                EWA_Filter_Compute(&altitude_ewa, (double)int_mavlink_height);
            }
            //UARTprintf("\n int_distance= %d", int_distance);
        }
    }
    else if(altitude_controler.altitude_source==1)
    {
        if(ultrasonic_update_flag)
        {
            ultrasonic_update_flag = false;
            if(ultrasonic_distance>=30 && ultrasonic_distance<2500)
            {
                //滤波
                EWA_Filter_Compute(&altitude_ewa, ultrasonic_distance);
            }
        }
    }
    //camera data
    u3Data_handle();
}

//串口初始化
void ConfigureUART0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 9600, 16000000);
}

//指示灯初始化
void LED_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_3);
}

//启动按键初始化
void Key_PF4_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

//启动按键检测
void Key_PF4_Pros(void)
{
    uint8_t ReadPin;
    ReadPin=GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
    if((ReadPin&GPIO_PIN_4)!=GPIO_PIN_4)
    {
        ROM_SysCtlDelay(20*(40000000/3000));
        ReadPin=GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
        if((ReadPin&GPIO_PIN_4)!=GPIO_PIN_4)
        {
            aircraft_start_flag=true;
        }
     while(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));
     }
}

void Key_PF0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

void Key_PF0_Pros(void)
{
    uint8_t ReadPin;
    ReadPin=GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
    if((ReadPin&GPIO_PIN_0)!=GPIO_PIN_0)
    {
        SysCtlDelay(20*(40000000/3000));
        ReadPin=GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
        if((ReadPin&GPIO_PIN_0)!=GPIO_PIN_0)
        {
            aircraft_mission++;
            if(aircraft_mission>AIRCRAFT_MISSION_NUM)
            {
                aircraft_mission=1;
            }
        }
        while(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0));
     }
}

//ppm通道初始化
void PPM_Channal_Init(void)
{
    ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_LOW;
    ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
    ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
    ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
    ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_ALTITUDE_HOLD_MODE;
    ppm_data.ch_val[5]=1500;
    ppm_data.ch_val[6]=1500;
    ppm_data.ch_val[7]=1500;
    ppm_encoder_set_data(&ppm_data);
}

//飞行器解锁
void Aircraft_Unlock(void)
{
    ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIN;
    ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MAX;
    ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
    ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
    ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_ALTITUDE_HOLD_MODE;
    ppm_data.ch_val[5]=1500;
    ppm_data.ch_val[6]=1500;
    ppm_data.ch_val[7]=1500;
    ppm_encoder_set_data(&ppm_data);
}

//ppm波初始输出
void Aircraft_Base_Output(void)
{
    ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIDDLE;
    ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
    ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
    ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
    ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_ALTITUDE_HOLD_MODE;
    ppm_data.ch_val[5]=1500;
    ppm_data.ch_val[6]=1500;
    ppm_data.ch_val[7]=1500;
    ppm_encoder_set_data(&ppm_data);
}

//侧翻保护
void RolloverProtection(void)
{
    if(int_pitch>60||int_pitch<-60||int_roll>60||int_roll<-60)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
        altitude_controler.altitude_hold_flag = false;
        position_controler.position_hold_flag = false;
        //power min
        ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIN;
        ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
        ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
        ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
        //自稳模式
        ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_STABILIZE_MODE;
        ppm_encoder_set_data(&ppm_data);
    }
}

//主控制定时器初始化
void Timer1A_Init(void)//5ms
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerClockSourceSet(TIMER1_BASE,TIMER_CLOCK_SYSTEM);
    ROM_TimerPrescaleSet(TIMER1_BASE,TIMER_A,8-1);//预分频
    TimerConfigure(TIMER1_BASE,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);//具体工作方式
    ROM_TimerLoadSet(TIMER1_BASE,TIMER_A,(ROM_SysCtlClockGet())/((ROM_TimerPrescaleGet(TIMER1_BASE,TIMER_A)+1)/200)-1);//装载值
    IntMasterEnable();//使能总中断
    ROM_IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);//计数到达装载值则中断
    //IntPrioritySet(INT_TIMER2A, 0x0);
    TimerEnable(TIMER1_BASE,TIMER_A);//计数器跟随系统时钟
}

//主控制
void Timer1AIntHandler(void)
{
    uint32_t ui32Status;
    static uint32_t ultrasonic_count = 1;
    static uint32_t altitude_hold_count = 1;
    static uint32_t position_hold_count = 6;
    ui32Status=TimerIntStatus(TIMER1_BASE,true);
    TimerIntClear(TIMER1_BASE,ui32Status);
    //protect
    RolloverProtection();
    //uart receive
    UART1_Data_Pros();
    //mavlink data/sonic data/camera data
    AircraftDataReceive();
    //sonic trig
    if(ultrasonic_count>6)//30ms
    {
        ultrasonic_count = 1;
        SonicTrig();//15us
    }
    //altitude hold
    if(altitude_hold_count>12)//60ms处理一次
    {
        altitude_hold_count = 1;
        Altitude_Control();
    }
    //position hold
    if(position_hold_count>12)
    {
        position_hold_count = 1;
        Position_Control();
    }
    ultrasonic_count ++;
    altitude_hold_count ++;
    position_hold_count ++;
}

void PositionHoldLand(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
    altitude_controler.altitude_hold_flag=false;
    position_controler.position_hold_flag=true;
    //power 35%
    ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIDDLE-150;
    ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
    ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
    ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
    //定高模式
    ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_ALTITUDE_HOLD_MODE;
    ppm_encoder_set_data(&ppm_data);
}

void Timer2AIntHandler(void)
{
    uint32_t ui32Status;
    static bool hover_count_start = false;
    static uint32_t hover_count = 0;
    ui32Status=TimerIntStatus(TIMER2_BASE,true);
    TimerIntClear(TIMER2_BASE,ui32Status);
    if(AltitudeGet()>=altitude_controler.target_altitude-50)
    {
        hover_count_start=true;
    }
    if(aircraft_mission==1||aircraft_mission==2)
    {
        if(hover_count>=1000)
        {
            hover_count_start=false;
            PositionHoldLand();
        }
    }
    if(hover_count_start)
    {
        hover_count++;
    }
}

void Timer2A_Init(void)//5ms
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerClockSourceSet(TIMER2_BASE,TIMER_CLOCK_SYSTEM);
    TimerPrescaleSet(TIMER2_BASE,TIMER_A,8-1);//预分频
    TimerConfigure(TIMER2_BASE,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);//具体工作方式
    TimerLoadSet(TIMER2_BASE,TIMER_A,(SysCtlClockGet())/((TimerPrescaleGet(TIMER2_BASE,TIMER_A)+1)/200)-1);//装载值
    IntMasterEnable();//使能总中断
    TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT);//计数到达装载值则中断
    IntPrioritySet(INT_TIMER2A, 0x0);
    TimerIntRegister(TIMER2_BASE, TIMER_A, Timer2AIntHandler);
    TimerEnable(TIMER2_BASE,TIMER_A);//计数器跟随系统时钟
}

//串口指令数据处理
void UART1_Data_Pros(void)
{
    if(UART1_Updated_Flag)
    {
        //地面指令
        if(UART1_Sender_Address==0xD1)
        {
            //降落
            if(UART1_Rx_Data[0]==0xDE&&UART1_Rx_Data[1]==0x01)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
                altitude_controler.altitude_hold_flag=false;
                position_controler.position_hold_flag=false;
                //power 35%
                ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIDDLE-150;
                ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
                ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
                ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
                ppm_encoder_set_data(&ppm_data);
            }
            //迫降
            if(UART1_Rx_Data[0]==0xDE&&UART1_Rx_Data[1]==0x02)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
                altitude_controler.altitude_hold_flag=false;
                position_controler.position_hold_flag=false;
                //power min
                ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIN;
                ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
                ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
                ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
                //自稳模式
                ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_STABILIZE_MODE;
                ppm_encoder_set_data(&ppm_data);
            }
            //上锁
            if(UART1_Rx_Data[0]==0xDE&&UART1_Rx_Data[1]==0x03)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1|GPIO_PIN_2);
                altitude_controler.altitude_hold_flag=false;
                position_controler.position_hold_flag=false;
                //power min
                ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIN;
                ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIN;
                ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
                ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
                ppm_encoder_set_data(&ppm_data);
            }
        }
        //car
        else if(UART1_Sender_Address==0xD2)
        {
            if(UART1_Rx_Data[0]==0x88&&UART1_Rx_Data[1]==0x81)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
                altitude_controler.altitude_hold_flag=false;
                position_controler.position_hold_flag=false;
                //power 35%
                ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIDDLE-150;
                ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
                ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE-15;
                ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
                //定高模式
                ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_ALTITUDE_HOLD_MODE;
                ppm_encoder_set_data(&ppm_data);
            }
        }
    }
}

//电调校准主函数
void CalibrationMain(void)
{
    int power_state=1;
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL |SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//80Mhz
    FPUEnable();//浮点运算
    FPULazyStackingEnable();
    LED_Init();//指示灯初始化
    ConfigureUART0();//开串口
    ppm_encoder_init();
    //
    ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MAX;
    ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
    ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
    ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
    ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_ALTITUDE_HOLD_MODE;
    ppm_data.ch_val[5]=1500;
    ppm_data.ch_val[6]=1500;
    ppm_data.ch_val[7]=1500;
    ppm_encoder_set_data(&ppm_data);
    Key_PF4_Init();
    while(!aircraft_start_flag)
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1);
        ROM_SysCtlDelay(50*(SysCtlClockGet()/3000));
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_3);
        ROM_SysCtlDelay(50*(SysCtlClockGet()/3000));
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_2);
        ROM_SysCtlDelay(50*(SysCtlClockGet()/3000));
        //检测启动键
        Key_PF4_Pros();
    }
    //power min
    ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIN;
    ppm_data.ch_val[PPM_YAW_CHANNAL]=PPM_YAW_MIDDLE;
    ppm_data.ch_val[PPM_PITCH_CHANNAL]=PPM_PITCH_MIDDLE;
    ppm_data.ch_val[PPM_ROLL_CHANNAL]=PPM_ROLL_MIDDLE;
    ppm_data.ch_val[PPM_STATION_CHANNAL]=PPM_ALTITUDE_HOLD_MODE;
    ppm_data.ch_val[5]=1500;
    ppm_data.ch_val[6]=1500;
    ppm_data.ch_val[7]=1500;
    ppm_encoder_set_data(&ppm_data);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
    //6s
    ROM_SysCtlDelay(8000*(SysCtlClockGet()/3000));
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
    while(1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_3);
        ROM_SysCtlDelay(50*(SysCtlClockGet()/3000));
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_3|GPIO_PIN_2);
        ROM_SysCtlDelay(50*(SysCtlClockGet()/3000));
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_2|GPIO_PIN_1);
        ROM_SysCtlDelay(50*(SysCtlClockGet()/3000));
        if(power_state==1)
        {
            ppm_data.ch_val[PPM_POWER_CHANNAL]=ppm_data.ch_val[PPM_POWER_CHANNAL]+25;
            if(ppm_data.ch_val[PPM_POWER_CHANNAL]>=PPM_POWER_MAX)
            {
                ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MAX;
                power_state=-1;
            }
            ppm_encoder_set_data(&ppm_data);
        }
        else if(power_state==-1)
        {
            ppm_data.ch_val[PPM_POWER_CHANNAL]=ppm_data.ch_val[PPM_POWER_CHANNAL]-25;
            if(ppm_data.ch_val[PPM_POWER_CHANNAL]<=PPM_POWER_MIN)
            {
                ppm_data.ch_val[PPM_POWER_CHANNAL]=PPM_POWER_MIN;
                power_state=0;
            }
            ppm_encoder_set_data(&ppm_data);
        }
    }
}

//启动检测
void Aircraft_Startup_Detection(void)
{
    while(!aircraft_start_flag||aircraft_mission==0)
    {
        Key_PF0_Pros();
        if(aircraft_mission!=0)
        {
            //检测启动键
            Key_PF4_Pros();
        }
        switch(aircraft_mission)
        {
        case 0:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            break;
        case 1:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            break;
        case 2:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_2);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            break;
        case 3:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_3);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            break;
        case 4:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_2);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            break;
        case 5:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_3);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            break;
        case 6:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_3|GPIO_PIN_2);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            break;
        case 7:
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
            ROM_SysCtlDelay(20*(SysCtlClockGet()/3000));
            break;
        }
    }
}

//飞行器主函数
void AircraftMain(void)
{
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL |SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);//80Mhz
    FPUEnable();//浮点运算
    FPULazyStackingEnable();
    LED_Init();//指示灯初始化
    ConfigureUART0();//开串口
    UltrasonicConfigure();//开超声波
    AltitudeControlerInit();//高度控制器初始化
    PositionControlerInit();//定点控制器初始化
    ppm_encoder_init();//开ppm波
    PPM_Channal_Init();//ppm波通道值初始化
    Key_PF4_Init();//启动按键初始化
    Key_PF0_Init();
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
    ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0x0);
    //启动检测
    Aircraft_Startup_Detection();
    //准备启动
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2,GPIO_PIN_3|GPIO_PIN_2);
    ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2,0x0);
    //飞行器解锁
    Aircraft_Unlock();
    ROM_SysCtlDelay(3500*(SysCtlClockGet()/3000));
    //解锁完成
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2,GPIO_PIN_1|GPIO_PIN_2);
    ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2,0x0);
    //油门输出一个基值，防止飞行器再次上锁
    Aircraft_Base_Output();
    ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
    //开串口一接受指令
    Recive_UART1_Config();
    //开Mavlink数据接收
    Mavlink_DateInit();
    //camera
    Recive_UART3_Config();
    //主控制定时器开启
    Timer1A_Init();
    //2.5ms
    ROM_SysCtlDelay(25*(SysCtlClockGet()/30000));
    //任务调度定时器
    Timer2A_Init();
    //开启所有中断
    IntMasterEnable();
    //启动完成
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0);
    //开始定高
    altitude_controler.altitude_hold_flag = true;
    //开始定点
    position_controler.position_hold_flag = true;
    while(1)
    {
        UARTprintf("%4d %4d %4d %4d %4d %4d %4d\n", int_mavlink_height, (int)altitude_ewa.output, \
                   (int)(100.0*(ppm_data.ch_val[PPM_POWER_CHANNAL]-PPM_POWER_MIN)/(PPM_POWER_MAX-PPM_POWER_MIN)), \
                   x_hat, \
                   (int)(100.0*(ppm_data.ch_val[PPM_ROLL_CHANNAL]-PPM_ROLL_MIN)/(PPM_ROLL_MAX-PPM_ROLL_MIN)), \
                   y_hat, \
                   (int)(100.0*(ppm_data.ch_val[PPM_PITCH_CHANNAL]-PPM_PITCH_MIN)/(PPM_PITCH_MAX-PPM_PITCH_MIN)));
        ROM_SysCtlDelay(100*(SysCtlClockGet()/3000));
        if(aircraft_mission==1)
        {
            Camera_Hold_Black_Blob();
        }
        else if(aircraft_mission==2)
        {
            Camera_Hold_Red_Blob();
        }
        else if(aircraft_mission==3)
        {
            Camera_Hold_Red_Blob();
        }
    }
}

void main(void)
{
    if(CALIBRATION_MODE==true)
    {
        CalibrationMain();
    }
    else
    {
        AircraftMain();
    }
}
