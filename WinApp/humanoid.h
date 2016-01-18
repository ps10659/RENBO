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
#include "util.h"

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




	I32_T		MotorState;

	// used by event waiting
	BOOL_T		Flag_StartMasterDone;
	BOOL_T		Flag_SetMotorParameterDone;
	BOOL_T		Flag_SetCurrPosHomeDone;

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



	// motor status
	F64_T		actualTheta[TOTAL_AXIS];

}WIN32_DAT;




string file_list[20]; //20 file buffer for the list
string walking_traj_dir = "C:..\\..\\WalkingTrajectories\\";
LPCSTR walking_trajectory_file = "C:..\\..\\WalkingTrajectories\\*.txt";

string pose_dir = "C:..\\..\\Poses\\";
LPCSTR pose_file = "C:..\\..\\Poses\\*.txt";
double Pos_home[TOTAL_AXIS] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0};
double Pos_temp[TOTAL_AXIS];
double Pos_walking_inital[TOTAL_AXIS];


void UpdataAllActualTheta(WIN32_DAT *pWinData);
void PrintAllActualTheta(WIN32_DAT *pWinData);

void printWalkingTrajectories(WIN32_DAT *pWinData);
void WriteWalkingTrajectories(WIN32_DAT *pWinData);



//
void InitPwindata(WIN32_DAT *pWinData);
void GenerateCubicPolyVec(WIN32_DAT *pWinData);
void UpdateWalkTraj();
void UpdateWalkTraj(int traj_num);
void UpdatePpPose();
void UpdatePpPose(int pose_num);
void ImportParameterTxt();
void PrintImportParameterTxt();

// 
void StartMaster();
void SetMotorParam();
void SetCurrPosHome();
void ServoOff();
void CloseMaster();

// control function
void NoTorque();
void HoldPos();
void HOMING_MoveToHomeSensor();
void PP_Move_rad(double *PP_targetTheta, double timePeriod, bool wait_until_reach);
void PP_Move_deg(double *PP_targetTheta, double timePeriod, bool wait_until_reach);
void CSP_Run();