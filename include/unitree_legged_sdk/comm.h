/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

#include <stdint.h>
#include <array>
namespace UNITREE_LEGGED_SDK 
{

	constexpr int HIGHLEVEL = 0x00;
	constexpr int LOWLEVEL  = 0xff;
	constexpr double PosStopF = (2.146E+9f);
	constexpr double VelStopF = (16000.0f);

#pragma pack(1)

	typedef struct
	{
		float x;
		float y;
		float z;
	} Cartesian;

	typedef struct
	{
		std::array<float, 4> quaternion;               // quaternion, normalized, (w,x,y,z)
		std::array<float, 3> gyroscope;                // angular velocity （unit: rad/s)    (raw data)
		std::array<float, 3> accelerometer;            // m/(s2)                             (raw data)
		std::array<float, 3> rpy;                      // euler angle（unit: rad)
		int8_t temperature;
	} IMU;                                 // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

	typedef struct
	{
		uint8_t r;
		uint8_t g;
		uint8_t b;
	} LED;                                 // foot led brightness: 0~255

	typedef struct
	{
		uint8_t mode;                      // motor working mode 
		float q;                           // current angle (unit: radian)
		float dq;                          // current velocity (unit: radian/second)
		float ddq;                         // current acc (unit: radian/second*second)
		float tauEst;                      // current estimated output torque (unit: N.m)
		float q_raw;                       // current angle (unit: radian)
		float dq_raw;                      // current velocity (unit: radian/second)
		float ddq_raw;
		int8_t temperature;                // current temperature (temperature conduction is slow that leads to lag)
		std::array<uint32_t, 2> reserve;
	} MotorState;                          // motor feedback

	typedef struct
	{
		uint8_t mode;                      // desired working mode
		float q;                           // desired angle (unit: radian) 
		float dq;                          // desired velocity (unit: radian/second)
		float tau;                         // desired output torque (unit: N.m)
		float Kp;                          // desired position stiffness (unit: N.m/rad )
		float Kd;                          // desired velocity stiffness (unit: N.m/(rad/s) )
		std::array<uint32_t, 3> reserve;
	} MotorCmd;                            // motor control

	typedef struct
	{
		uint8_t levelFlag;                 // flag to distinguish high level or low level
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN; 
		uint8_t bandWidth;
		IMU imu;
		std::array<MotorState, 20> motorState;
		std::array<int16_t, 4> footForce;              // force sensors
		std::array<int16_t, 4> footForceEst;           // force sensors
		uint32_t tick;                     // reference real-time from motion controller (unit: us)
		std::array<uint8_t, 40> wirelessRemote;        // wireless commands
		uint32_t reserve;
		uint32_t crc;
	} LowState;                            // low level feedback

	typedef struct 
	{
		uint8_t levelFlag;
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN;
		uint8_t bandWidth;
		std::array<MotorCmd, 20> motorCmd;
		std::array<LED, 4> led;
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} LowCmd;                              // low level control

	typedef struct
	{
		uint8_t levelFlag;
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN;
		uint8_t bandWidth;
		uint8_t mode;
		float progress;
		IMU imu;
		uint8_t gaitType;                  // 0.idle  1.trot  2.trot running  3.climb stair
		float footRaiseHeight;             // (unit: m, default: 0.08m), foot up height while walking
		std::array<float, 3> position;                 // (unit: m), from own odometry in inertial frame, usually drift
		float bodyHeight;                  // (unit: m, default: 0.28m),
		std::array<float, 3> velocity;                 // (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
		float yawSpeed;                    // (unit: rad/s), rotateSpeed in body frame        
		std::array<Cartesian, 4> footPosition2Body;    // foot position relative to body
		std::array<Cartesian, 4> footSpeed2Body;       // foot speed relative to body
		std::array<int16_t, 4> footForce;
		std::array<int16_t, 4> footForceEst;
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} HighState;                           // high level feedback

	typedef struct
	{
		uint8_t levelFlag;
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN;
		uint8_t bandWidth;
		uint8_t mode;                       // 0. idle, default stand  1. force stand (controlled by dBodyHeight + ypr)
											// 2. target velocity walking (controlled by velocity + yawSpeed)
											// 3. target position walking (controlled by position + ypr[0])
											// 4. path mode walking (reserve for future release)
											// 5. position stand down. 
											// 6. position stand up 
											// 7. damping mode 
											// 8. recovery stand
											// 9. backflip
											// 10. jumpYaw
											// 11. straightHand
											// 12. dance1
											// 13. dance2
											
		uint8_t gaitType;                  // 0.idle  1.trot  2.trot running  3.climb stair
		uint8_t speedLevel;                // 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
		float footRaiseHeight;             // (unit: m, default: 0.08m), foot up height while walking
		float bodyHeight;                  // (unit: m, default: 0.28m),
		std::array<float, 2> postion;                  // (unit: m), desired position in inertial frame
		std::array<float, 3> euler;                    // (unit: rad), roll pitch yaw in stand mode
		std::array<float, 2> velocity;                 // (unit: m/s), forwardSpeed, sideSpeed in body frame
		float yawSpeed;                    // (unit: rad/s), rotateSpeed in body frame
		std::array<LED, 4> led;
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} HighCmd;                             // high level control

#pragma pack()

	typedef struct     
	{
		unsigned long long TotalCount;     // total loop count
		unsigned long long SendCount;      // total send count
		unsigned long long RecvCount;      // total receive count
		unsigned long long SendError;      // total send error 
		unsigned long long FlagError;      // total flag error 
		unsigned long long RecvCRCError;   // total reveive CRC error	
		unsigned long long RecvLoseError;  // total lose package count	
	} UDPState;                            // UDP communication state

	constexpr int HIGH_CMD_LENGTH   = (sizeof(HighCmd));
	constexpr int HIGH_STATE_LENGTH = (sizeof(HighState));
	
}

#endif
