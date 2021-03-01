/*
 * mavlink.h
 *
 *  Created on: 2019年1月20日
 *      Author: wmy
 */

#ifndef MAVLINK_H_
#define MAVLINK_H_

#define USE_ATTITUDE
//#include "r_cg_macrodriver.h"
//#include "r_cg_port.h"

#define MAVLINK_STX  0xFE
//#define MSG_SONAR_LENGTH 8
//#define MSG_SONAR_ID     173
//#define MSG_HEIGHT_LENGTH 16

//***ATTITUDE
#ifdef USE_ATTITUDE
#define MSG_ATTITUDE_ID  30
#define MSG_ATTITUDE_PAYLOAD_LEN 28
#define MSG_ATTITUDE_WHOLEN 36
//***ATTITUDE #define END
typedef struct MAVLINK_ATTITUDE_PAYLOAD
{
   //uint32_t time_boot_ms;  //user needn't
   float roll;
   float pitch;
   float yaw;
   float rollspeed ;
   float pitchspeed ;
   float yawspeed ;
}Attitude_Payload;
#endif
//****HEARTBEAT
//#define MAVLINK_MSG_ID_HEARTBEAT 0
//#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
//****HEARTBEAT #define END

//****AHRS2
#ifdef USE_AHRS
#define MSG_ID_AHRS2 178
#define MSG_AHRS2_PAYLOAD_LEN 24
//****AHRS2 #define END
typedef struct MAVLINK_AHRS2_PAYLOAD
{
 float roll; ///< Roll angle (rad)
 float pitch; ///< Pitch angle (rad)
 float yaw; ///< Yaw angle (rad)
 float altitude; ///< Altitude (MSL)
 int32_t lat; ///< Latitude in degrees * 1E7
 int32_t lng; ///< Longitude in degrees * 1E7
}Ahrs2_Payload;
#endif

//***GLOBAL_POSITION
#ifdef USE_GLOBAL_POSITION
#define MSG_ID_GLOBAL_POSITION_INT 33
#define MSG_GLOBAL_POSITION_PAYLOAD_LEN 28
//****GLOBAL_POSITION #define END
typedef struct MAVLINK_GLOBAL_POSITION_INT_PAYLOAD
{
     //uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)  //user needn't
     int32_t lat; ///< Latitude, expressed as * 1E7
     int32_t lon; ///< Longitude, expressed as * 1E7
     int32_t alt; ///< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
     int32_t relative_alt; ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
     int16_t vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
     int16_t vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
     int16_t vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
     uint16_t hdg; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX

}GlobalPos_Payload;
#endif
//****Rangfinder
#define MSG_ID_RANGEFINDER 173
#define MSG_RANGEFINDER_LEN 8
#define MSG_RANGEFINDER_WHOLEN 16
//****Rangfinder #define END
typedef struct MAVLINK_RANGEFINDER_PAYLOAD
{
 float distance; ///< distance in meters
 float voltage; ///< raw voltage if available, zero otherwise
}Rangefinder_Payload;

//float  uchar_to_float(uint8_t *buffer);
//void calculate_angle(void);
//void  calculate_height(void);
extern void Mavlink_DateInit(void);
extern void MavLink_Data_Get(void);

#define PAYLOAD_BUF_NUM 2          //姿态信息的数组的维度

extern bool newAttiFlag;
extern bool newHeightFlag;
extern int int_pitch, int_roll, int_yaw, int_mavlink_height;
extern float pitch, roll, yaw, mavlink_height;

extern uint8_t payload_buf_index ; //姿态信息的数组的索引值

extern Attitude_Payload Source_attitude_payload[PAYLOAD_BUF_NUM] ;
extern Rangefinder_Payload Source_Rangefinder_payload ;



#endif /* MAVLINK_H_ */
