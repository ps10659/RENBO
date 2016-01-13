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
#define TOTAL_MOTION_NUMBER 6
#define MAX_MOTION_TIME_FRAME 5000 // --> max motion time period = MAX_MOTION_TIME_FRAME * cycletime
#define SHM_NAME "Share Memory"
#define EVN_NUM 21

// constant
#define FSM_STATE int
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


// CONTROL_STATE
#define BEGINNING 0
#define START_MASTER_AND_SLAVES 1
#define SET_MOTOR_PARAMETERS 2
#define SET_CURR_POS_HOME 3
#define HOMING 4
#define GO_HOME 5
#define STOP_AND_HOLD 6
#define STOP_BUT_SOFT 7
#define HOLD 8
#define SERVO_OFF 9
#define CLOSE_MASTER 10

#define HOMING_MODE 21
#define	HOMING_CHECK_IK_LIMIT 22
#define HOMING_RUN 23

#define PP_MODE 31
#define PP_CHECK_IK_LIMIT 32
#define PP_RUN 33

#define CSP_MODE 41
#define CSP_CHECK_IK_LIMIT 42
#define CSP_RUN 43

#define WRITE_FILE 99



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

	// state, flags and switches
	FSM_STATE	currentState;
	BOOL_T		stateTransitionFlag;	// 1/0: transitioning/arrived new state
	BOOL_T		setServoOnFlag;
	BOOL_T		setServoOffFlag;
	BOOL_T		holdSwitch;
	BOOL_T		setTargetTorqueSwitch;	// 1/0: torque on/off
	BOOL_T		Flag_HoldPosSaved;
	BOOL_T		home35CompleteFlag;
	BOOL_T		updateAllActualThetaFlag;
	BOOL_T		resetCntFlag;

	BOOL_T		Flag_StartMasterDone;
	BOOL_T		Flag_SetMotorParameterDone;
	BOOL_T		Flag_SetCurrPosHomeDone;



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


// event
LPCSTR EVN_NAME[EVN_NUM] = 
{
	"BEGINNING",
	"START_MASTER_AND_SLAVES",
	"SET_MOTOR_PARAMETERS",
	"SET_CURR_POS_HOME",
	"HOLD"
	"HOMING",
	"GO_HOME",
	"STOP_AND_HOLD",
	"STOP_BUT_SOFT",
	"SERVO_OFF",
	"CLOSE_MASTER",

	"HOMING_MODE",
	"HOMING_CHECK_IK_LIMIT",
	"HOMING_RUN",

	"PP_MODE",
	"PP_CHECK_IK_LIMIT",
	"PP_RUN",

	"CSP_MODE",
	"CSP_CHECK_IK_LIMIT",
	"CSP_RUN",

	"WRITE_FILE"
};




//  Add Function prototypes Here
void __RtCyclicCallback( void *UserDataPtr );
void __RtEventCallback( void *UserDataPtr, U32_T EventCode ); 
void __RtErrorCallback( void *UserDataPtr, I32_T ErrorCode );

void	StartMaster(USER_DAT *pData);
void	CloseMaster(USER_DAT *pData);
RTN_ERR SetMotorParameters(USER_DAT *pData);
RTN_ERR MotorType_2342(CANAxis_T Axis);
RTN_ERR MotorType_2619(CANAxis_T Axis);
RTN_ERR MotorType_2642(CANAxis_T Axis);
RTN_ERR MotorType_3257(CANAxis_T Axis);
RTN_ERR MotorType_3863(CANAxis_T Axis);
RTN_ERR MotorType_3890(CANAxis_T Axis);
void HomingMethod35(USER_DAT *pData);

void HOMING_UpdateCbTargetTheta(F64_T *CB_targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *homeSensorValue, I32_T *HOMING_cnt);
void PP_UpdateCbTargetTheta(F64_T *CB_targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *CSP_cnt);
void CSP_UpdateCbTargetTheta(F64_T *CB_targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *CSP_cnt);

I16_T TargetTorqueTrimming(F64_T tempTorque);




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