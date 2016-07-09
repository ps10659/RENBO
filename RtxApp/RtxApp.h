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
#include <tchar.h>
#include <wchar.h>
#include <Eigen/Dense>
#include "Kinematics.h"

#include "NexECMRtx.h"
#include "NexCoeMotion.h"
#include "EcErrors.h"
#include "math.h"

// defines
#define TOTAL_AXIS 33
#define	MAX_WALKING_TIMEFRAME 50000
#define MAX_MOTION_TIME_FRAME 10000
#define PP_QUEUE_SIZE 100
#define TOTAL_MOTION_NUMBER 6
#define SHM_NAME "Share Memory"
#define EVN_NUM 4

// constant
#define PI 3.14159265359
#define MICROSECOND_TO_SECOND 0.000001
#define SECOND_TO_MICROSECOND 1000000

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
#define MotorState_FtsTest 5
#define MotorState_OPG 6


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


// share memory
typedef struct
{
	U16_T		masterId;
	CANAxis_T	axis[TOTAL_AXIS];
	I32_T       cycleTime;

	U8_T		*InputProcessDataPtr; 
	U32_T		InPDSizeInByte; 
	U8_T		*OutputProcessDataPtr; 
	U32_T		OutPDSizeInByte;



	// motor status
	I32_T		MotorState;
	F64_T		actualTheta[TOTAL_AXIS];
	I16_T		supportState;	// 0:left 1:right 2:double 3:none

	// used by event waiting
	BOOL_T		Flag_StartMasterDone;
	BOOL_T		Flag_SetMotorParameterDone;

	// used in cyclic callback funciton
	BOOL_T		Flag_ResetCnt;
	BOOL_T		Flag_ResetCbErrorTheta;
	BOOL_T		Flag_ServoOn;
	BOOL_T		Flag_ServoOff;
	BOOL_T		Flag_HoldPosSaved;
	BOOL_T		Flag_UpdateActualTheta;
	BOOL_T		Flag_PpReachTarget;
	BOOL_T		Flag_CspFinished;
	BOOL_T		Flag_AllHomeSensorReached;
	BOOL_T		Flag_HomeSensorReached[TOTAL_AXIS];
	BOOL_T		Flag_SetCurrPosHome;
	BOOL_T		Flag_ResetT_cog;

	// motor parameters
	F64_T		motorTorqueSwitch[TOTAL_AXIS];
	F64_T		Kp[TOTAL_AXIS];
	F64_T		Ki[TOTAL_AXIS];
	F64_T		Kd[TOTAL_AXIS];


	// HOMING related variables
	F64_T		HOMING_homePositionOffset[TOTAL_AXIS];
	F64_T		HOMING_homeSensorTheta[TOTAL_AXIS];
	

	// PP related variables
	F64_T		CubicPolyVec[MAX_MOTION_TIME_FRAME];
	F64_T		PP_Queue_TargetTheta[PP_QUEUE_SIZE][TOTAL_AXIS];
	F64_T		PP_Queue_TimePeriod[PP_QUEUE_SIZE];
	I32_T		PP_Queue_Rear;
	I32_T		PP_Queue_Front;

	//CSP related variables
	F64_T		WalkingTrajectories[MAX_WALKING_TIMEFRAME][TOTAL_AXIS];
	F64_T		ActualWalkingTrajectories[MAX_WALKING_TIMEFRAME][TOTAL_AXIS];
	I32_T		walkingTimeframe;
	F64_T		walkingSpeed;
	F64_T		S_Step;
	I32_T		adaptive_cnt;

	// OPG related variables
	I16_T		sup_leg;	
	I16_T		curr_state;	
	I16_T		next_state;	
	I16_T		next_state_cmd;		// -2: before started 
									// -1: stop 
									//  0: begin to stop
									//  1: begin to start
									//  5: walk in the same place
									//  8: walk forward
									//  2: walk backward
	F64_T		leg_swing_xy_vec[2500];	
	F64_T		leg_swing_z_vec[2500];	// 暫時用GenerateCubicPolyVec的3rd order spline, 有需要再改
	F64_T		target_cog[4];
	F64_T		target_left_foot[4];
	F64_T		target_right_foot[4];
	F64_T		left_foot_theta[6];
	F64_T		right_foot_theta[6];
	F64_T		l_leg_gc[6];	// temp for checking
	F64_T		r_leg_gc[6];	// temp
	F64_T		actual_cog[3];	// temp
	F64_T		actual_left_foot[3];	// temp
	F64_T		actual_right_foot[3];	// temp
	BOOL_T		Flag_break_while;
	BOOL_T		Flag_ResetStaticInOPG;

	F64_T		step_time;
	F64_T		cog_height_for_omega;
	F64_T		cog_height_for_IK;
	F64_T		foot_distance;
	F64_T		step_length;
	F64_T		swing_leg_height;
	F64_T		gc_l_hip_pitch;
	F64_T		gc_r_hip_pitch;
	F64_T		gc_l_ankle_pitch;
	F64_T		gc_r_ankle_pitch;

	// force torque data
	F64_T mx[2];
	F64_T my[2];
	F64_T mz[2];
	F64_T fx[2];
	F64_T fy[2];
	F64_T fz[2];
	F64_T mx_offset[2];
	F64_T my_offset[2];
	F64_T fz_offset[2];
	
	F64_T zmp_lx;
	F64_T zmp_ly;
	F64_T zmp_rx;
	F64_T zmp_ry;
	F64_T zmp_x;
	F64_T zmp_y;
	F64_T zmp_unit;

	F64_T Fts_LRK;	//left ankle roll K
	F64_T Fts_LRC;	//left ankle roll C
	F64_T Fts_LPK;	//left ankle pitch K
	F64_T Fts_LPC;
	F64_T Fts_RRK;	//right ankle roll K
	F64_T Fts_RRC;
	F64_T Fts_RPK;
	F64_T Fts_RPC;

	F64_T FzThreshold;
	F64_T MxyThreshold;
	
	// temp
	I32_T DoubleSupport_cnt;
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


void StartMaster(USER_DAT *pData);
void CloseMaster(USER_DAT *pData);
void HomingMethod35(USER_DAT *pData);
void SaveHoldPos(F64_T *CB_targetTheta, F64_T *CB_actualTheta);
void HOMING_UpdateCbTargetTheta(F64_T *CB_targetTheta, F64_T *CB_actualTheta, I32_T *home_sensor_value, I32_T *HOMING_cnt);
void PP_UpdateCbTargetTheta(F64_T *targetTheta, F64_T *CB_actualTheta, I32_T *PP_cnt);
void CSP_UpdateCbTargetTheta(F64_T *CB_targetTheta, F64_T *CB_actualTheta, I32_T *CPS_cnt);
void Fts_UpdateCbTargetTheta(F64_T *CB_targetTheta, F64_T *CB_actualTheta, I32_T *CSP_cnt);
void OPG_UpdateTargetPose(Eigen::Matrix4d& T_cog, Eigen::Matrix4d& T_left_foot, Eigen::Matrix4d& T_right_foot, I32_T *OPG_cnt);
void PP_UpdateTargetPose(Eigen::Matrix4d& T_cog, Eigen::Matrix4d& T_left_foot, Eigen::Matrix4d& T_right_foot, I32_T *PP_cnt);

I16_T TargetTorqueTrimming(F64_T tempTorque);
void UpdateIK_FK(F64_T *CB_targetTheta, Eigen::Matrix4d& T_cog, Eigen::Matrix4d& T_left_foot, Eigen::Matrix4d& T_right_foot);
void GravityCompensation(F64_T *l_leg_gc, F64_T *r_leg_gc);
void MotorPosPidControl(F64_T *CB_targetTheta, F64_T *CB_actualTheta, USER_DAT *pData, F64_T *l_leg_gc, F64_T *r_leg_gc);

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


const int homeSensorReachValue[TOTAL_AXIS] = {-1, -1, -1, -1, 6,  -1, -1, -1, -1, -1,  6, -1, -1, -1, -1,  -1, -1, -1, -1, 7,  6, 6, 6, 6, 6,  6, 6, 6, 6, 6,  6, 6, 6};
const F64_T HOMING_maxVelocity[TOTAL_AXIS] = {0, 0, 0, 0, PI/36.0,  0, 0, 0, 0, 0,  -PI/36.0, 0, 0, 0, 0,  0, 0, 0, 0, PI/36.0,  PI/36.0,
												-PI/36.0, PI/36.0, -PI/36.0, PI/36.0, -PI/36.0, -PI/36.0, 
												PI/36.0, -PI/36.0, -PI/36.0, PI/36.0, -PI/36.0, PI/36.0};

F64_T maxErrorThetaSum = 5 * PI / 180.0;

#endif