#ifndef _RTXAPP_H_
#define _RTXAPP_H_

#include <windows.h>
#include <rtapi.h>
//#include "..\helloShare\helloShare.h"
//#include <stdio.h>
//#include <string.h>
//#include <ctype.h>
//#include <conio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <errno.h>

#include "NexECMRtx.h"
#include "NexCoeMotion.h"
#include "EcErrors.h"
#include "math.h"

// defines
#define TOTAL_AXIS 33
#define	MAX_WALKING_TIMEFRAME 50000
#define MAX_MOTION_TIME_FRAME 10000
#define TOTAL_MOTION_NUMBER 6
#define SHM_NAME "Share Memory"
#define EVN_NUM 4

// constant
#define PI 3.14159265359
#define MILISECOND_TO_SECOND 0.000001
#define SECOND_TO_MILISECOND 1000000

// motor
//#define L_Wrist_YAW 0
//#define L_Wrist_Pitch 1
//#define L_HAND 2
//#define L_SHOULDER_ROLL 3
//#define TRUNK_ROLL 4
//#define L_SHOULDER_PITCH 5
//#define HEAD_YAW 6
//#define HEAD_PITCH 7
//#define R_SHOULDER_PITCH 8
//#define LASER 9
//#define TRUNK_PITCH 10
//#define R_SHOULDER_ROLL 11
//#define R_HAND 12
//#define	L_ELBOW 13
//#define L_SHOULDER_YAW 14
//#define R_SHOULDER_YAW 15
//#define	R_ELBOW 16
//#define R_Wrist_Pitch 17
//#define R_Wrist_YAW 18
//#define TRUNK_YAW 19
//#define WAIST 20
//#define L_HIP_YAW 21
//#define L_HIP_ROLL 22
//#define L_HIP_PITCH 23
//#define L_KNEE 24
//#define L_ANKLE_PITCH 25
//#define L_ANKLE_ROLL 26
//#define R_HIP_YAW 27
//#define R_HIP_ROLL 28
//#define R_HIP_PITCH 29
//#define R_KNEE 30
//#define R_ANKLE_PITCH 31
//#define R_ANKLE_ROLL 32


// MotorState
#define MotorState_NoTq 0
#define MotorState_Hold 1
#define MotorState_Homing 2
#define MotorState_PP 3
#define MotorState_CSP 4


// event 
#define START_MASTER_AND_SLAVES 0
#define SET_MOTOR_PARAMETERS 1
#define SET_CURR_POS_HOME 2
#define CLOSE_MASTER 3

LPCSTR EVN_NAME[EVN_NUM] = 
{
	"START_MASTER_AND_SLAVES",
	"SET_MOTOR_PARAMETERS",
	"SET_CURR_POS_HOME",
	"CLOSE_MASTER",
};

// queue object used in MotorState_PP
struct PP
{
	double timePeriod;
	double theta[TOTAL_AXIS];
};

// shared memory
typedef struct
{
	U16_T		masterId;
	CANAxis_T	axis[TOTAL_AXIS];
	I32_T       cycleTime;

	U8_T		*InputProcessDataPtr; 
	U32_T		InPDSizeInByte; 
	U8_T		*OutputProcessDataPtr; 
	U32_T		OutPDSizeInByte;

	// walking trajectory
	F64_T		WalkingTrajectories[MAX_WALKING_TIMEFRAME][TOTAL_AXIS];
	F64_T		ActualWalkingTrajectories[MAX_WALKING_TIMEFRAME][TOTAL_AXIS];
	I32_T		walkingTimeframe;
	F64_T		CubicPolyVec[MAX_MOTION_TIME_FRAME];



	I32_T		MotorState;

	// used by event waiting
	BOOL_T		Flag_StartMasterDone;
	BOOL_T		Flag_SetMotorParameterDone;
	BOOL_T		Flag_SetCurrPosHomeDone;

	// used in cyclic callback funciton
	BOOL_T		Flag_ResetCnt;
	BOOL_T		Flag_ResetError;
	BOOL_T		Flag_ServoOn;
	BOOL_T		Flag_ServoOff;
	BOOL_T		Flag_HoldPosSaved;
	BOOL_T		Flag_UpdateActualTheta;



	// motor parameters
	F64_T		motorTorqueSwitch[TOTAL_AXIS];
	F64_T		Kp[TOTAL_AXIS];
	F64_T		Ki[TOTAL_AXIS];
	F64_T		Kd[TOTAL_AXIS];

	F64_T		walkingSpeed;

	// HOMING related variables
	F64_T		HOMING_initialTheta[TOTAL_AXIS];
	F64_T		HOMING_homePositionOffset[TOTAL_AXIS];
	F64_T		HOMING_homeSensorTheta[TOTAL_AXIS];
	BOOL_T		HOMING_homeSensorReachFlag[TOTAL_AXIS];
	BOOL_T		HOMING_allHomeSensorReachFlag;
	

	// PP related variables
	BOOL_T		PP_singleMovementCompleteFlag;
	F64_T		PP_initialTheta[TOTAL_AXIS];
	F64_T		PP_targetTheta[TOTAL_AXIS];
	F64_T		PP_splineVec[MAX_MOTION_TIME_FRAME];
	I32_T		PP_motionTimeFrame;
	I32_T		PP_currPointCnt;
	I32_T		PP_motionType;
	F64_T		PP_motionTimePeriod;
	I32_T		PP_totalPointCnt[TOTAL_MOTION_NUMBER];

	

	// motor status
	F64_T		actualTheta[TOTAL_AXIS];

}USER_DAT;


//  Add Function prototypes Here
void __RtCyclicCallback( void *UserDataPtr );
void __RtEventCallback( void *UserDataPtr, U32_T EventCode ); 
void __RtErrorCallback( void *UserDataPtr, I32_T ErrorCode );


RTN_ERR SetMotorParameters(USER_DAT *pData);
RTN_ERR MotorType_2342(CANAxis_T Axis);
RTN_ERR MotorType_2619(CANAxis_T Axis);
RTN_ERR MotorType_2642(CANAxis_T Axis);
RTN_ERR MotorType_3257(CANAxis_T Axis);
RTN_ERR MotorType_3863(CANAxis_T Axis);
RTN_ERR MotorType_3890(CANAxis_T Axis);

void HOMING_UpdateCbTargetTheta(F64_T *targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *homeSensorValue, I32_T *HOMING_cnt);
void PP_UpdateCbTargetTheta(F64_T *targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *CSP_cnt);
void CSP_UpdateCbTargetTheta(F64_T *targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *CSP_cnt);

void StartMaster(USER_DAT *pData);
void CloseMaster(USER_DAT *pData);
void HomingMethod35(USER_DAT *pData);
void SaveHoldPos(F64_T *targetTheta, F64_T *actualTheta);
I16_T TargetTorqueTrimming(F64_T tempTorque);

void MotorPosPidControl(F64_T *targetTheta, F64_T *actualTheta, USER_DAT *pData);

// global variables
static PVOID	location;

RTN_ERR		ret = 0;
U16_T       masterId = 0;
TClintParam clientParam;
I32_T       cycleTime = 2000; // cycle time for EtherCAT communication. Unit: microsecond


I16_T		motor_type[TOTAL_AXIS] = //{3890};
						{2642, 2342, 2619, 3863, 3890, 3863, 2642, 2642, 3863, 2642,
						 3890, 3863, 2619, 2642, 3257, 3257, 2642, 2342, 2642, 3890,
						 3890, 3890, 3890, 3890, 3890, 3890, 3890, 3890, 3890, 3890,
						 3890, 3890, 3890};

F64_T		resolution[TOTAL_AXIS] = //{4096}; // 記得把2*pi都刪掉,而且應該不是每個馬達都4096吧?????????每個馬達就是4096..
						{4096, 9162/PI, 64, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						 4096, 4096, 64, 4096, 4096, 4096, 4096, 9162/PI, 4096, 4096,
						 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						 4096, 4096, 4096};
						/*{4096, 64, 64, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						 4096, 4096, 64, 4096, 4096, 4096, 4096, 64, 4096, 4096,
						 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						 4096, 4096, 4096};*/
						
F64_T		reduction_ratio[TOTAL_AXIS] = //{3*160};  
						{2*100, 3*100, 1, 3*160, 3*160, 3*160, 246, 1*43, 3*160, 23*43,
						 3*160, 3*160, 1, 5*160, 2*160, 2*160, 5*160, 3*100, 2*100, 160,
						 3*160, 3*160, 4*160, 3*160, 2.5*160, 3*160, 3*160, 3*160, 4*160, 3*160,
						 2.5*160, 3*160, 3*160};

F64_T		motor_direction[TOTAL_AXIS] = //{1};//3,7還沒確定
						 {1, 1, 1, 1, -1, 1, 1, 1, 1, 1,
						  -1, 1, 1, 1, 1, 1, 1, 1, 1, -1,
						  -1, -1, 1, 1, -1, -1, -1, -1, 1, -1,
						  1, 1,-1};
F64_T		axis_theta_to_motor_resolution[TOTAL_AXIS];


const int homeSensorReachValue[TOTAL_AXIS] = 
	{-1, -1, -1, -1, 6, -1, -1, -1, -1, -1, 6, -1, -1, -1, -1, -1, -1, -1, -1, 7, 6, 
		6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, };

const F64_T HOMING_maxVelocity[TOTAL_AXIS] = 
	{0, 0, 0, 0, 2.0*PI/72.0, 0, 0, 0, 0, 0, -2.0*PI/72.0, 0, 0, 0, 0, 0, 0, 0, 0, 2.0*PI/72.0, 2.0*PI/72.0, 
		-2.0*PI/72.0, 2.0*PI/72.0, -2.0*PI/72.0, 2.0*PI/72.0, -2.0*PI/72.0, -2.0*PI/72.0, 
		2.0*PI/72.0, -2.0*PI/72.0, -2.0*PI/72.0, 2.0*PI/72.0, -2.0*PI/72.0, 2.0*PI/72.0, };

F64_T maxErrorThetaSum = 5 * PI / 180.0;

#endif