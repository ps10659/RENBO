#pragma once

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <tchar.h>
#include <windows.h>
#include "Rtapi.h"
#include <conio.h>
#include "nex_type.h"
#include "Nexecm.h"
#include "..\RtxApp\RtxApp.h"
#include <Eigen/Dense>
using namespace std;


// define
#define ESC_KEY 27


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

}WIN32_DAT;


#define UserDefineTotalPoint 1
#define GoHomeTotalPoint 1
#define SquatTotalPoint 5
#define WalkingInitialPoint 1
#define WalkingInitialReversePoint 1
#define ReadPoseTxtPoint 100



// Target Theta
F64_T UserDefineTheta[UserDefineTotalPoint][TOTAL_AXIS] = 
	{{0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0}};

F64_T HomeTheta[GoHomeTotalPoint][TOTAL_AXIS] = 
	{{0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0}};


F64_T SquatTheta[SquatTotalPoint][TOTAL_AXIS] = 
	{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
		0, 0, -60, 120, -60, 0,  0, 0, -60, 120, -60, 0}};

F64_T WalkingInitialTheta[WalkingInitialPoint][TOTAL_AXIS] =
	{{0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0, 
		0,0.0490,-22.3758,51.6053,-29.2296,-0.0490, 0,0.0490,-22.3758,51.6053,-29.2296,-0.0490
 }};

F64_T WalkingInitialReverseTheta[WalkingInitialReversePoint][TOTAL_AXIS] =
	{{0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,
		0,-0.0490,22.3758,-51.6053,29.2296,0.0490, 0,-0.0490,   22.3758,-51.6053,29.2296,0.0490}};


F64_T ReadPoseTxtTheta[ReadPoseTxtPoint][TOTAL_AXIS] = 
	{{0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0}};


// functions
bool TriggerEvent(int key, WIN32_DAT *pWinData);

bool goto_START_MASTER_AND_SLAVES(WIN32_DAT *pWinData);
bool goto_SET_MOTOR_PARAMETERS(WIN32_DAT *pWinData);
bool goto_SET_CURR_POS_HOME(WIN32_DAT *pWinData);
bool goto_HOMING(WIN32_DAT *pWinData);
bool goto_GO_HOME(WIN32_DAT *pWinData);
bool goto_STOP_AND_HOLD(WIN32_DAT *pWinData);
bool goto_STOP_BUT_SOFT(WIN32_DAT *pWinData);
bool goto_SERVO_OFF(WIN32_DAT *pWinData);
bool goto_CLOSE_MASTER(WIN32_DAT *pWinData);
bool goto_HOLD(WIN32_DAT *pWinData);

bool goto_HOMING_MODE(WIN32_DAT *pWinData);
bool goto_HOMING_RUN(WIN32_DAT *pWinData);

bool goto_PP_MODE(WIN32_DAT *pWinData);
bool goto_PP_CHECK_IK_LIMIT(WIN32_DAT *pWinData);
bool goto_PP_RUN(WIN32_DAT *pWinData);

bool goto_CSP_MODE(WIN32_DAT *pWinData);
bool goto_CSP_CHECK_IK_LIMIT(WIN32_DAT *pWinData);
bool goto_CSP_RUN(WIN32_DAT *pWinData);

bool goto_WRITE_FILE(WIN32_DAT *pWinData);



void UpdateSplineVector(WIN32_DAT *pWinData, int splineType, F64_T motionTimePeriod);
void UpdataAllActualTheta(WIN32_DAT *pWinData);
void PrintAllActualTheta(WIN32_DAT *pWinData);
void PrintUserDefinedTheta();
void UpdateAllKp(WIN32_DAT *pWinData);
void UpdateAllKd(WIN32_DAT *pWinData);
void PrintAllKp(WIN32_DAT *pWinData);
void PrintAllKd(WIN32_DAT *pWinData);


//void PP_User_Input(WIN32_DAT *pWinData);
//bool PP_setMotionType(WIN32_DAT *pWinData, F64_T targetTheta[][GoHomeTotalPoint], const int GoHomeTotalPoint);
void SetPP_targetTheta(WIN32_DAT *pWinData, int motionType, int currPointCnt);
void UpdataUserDefineData();

void UpdateWalkingTrajectories(WIN32_DAT *pWinData);
void printWalkingTrajectories(WIN32_DAT *pWinData);
void WriteWalkingTrajectories(WIN32_DAT *pWinData);

void ImportParameterTxt(WIN32_DAT *pWinData);
void PrintImportParameterTxt(WIN32_DAT *pWinData);
void WritePose(WIN32_DAT *pWinData);
void UpdateReadPoseTxtTheta(WIN32_DAT *pWinData);



void InitPwindata(WIN32_DAT *pWinData);
void StartMaster(WIN32_DAT *pWinData);
void SetMotorParam(WIN32_DAT *pWinData);
void SetCurrPosHome(WIN32_DAT *pWinData);
void HoldPos(WIN32_DAT *pWinData);