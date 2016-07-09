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
#include "force_sensor.h"

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
}WIN32_DAT;

ft_data fts;


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
void DisplayOptions();
void InitPwindata(WIN32_DAT *pWinData);
void GenerateCubicPolyVec(WIN32_DAT *pWinData);
void UpdateSplineVec(double *vec, int vec_length, int splineType);
void UpdateSwingVec(double *vec, int splineType);
void UpdateWalkTraj();
void UpdateWalkTraj(int traj_num);
void UpdatePpPose();
void UpdatePpPose(int pose_num);
void WritePpPose();
void ImportParameterTxt();
void PrintImportParameterTxt();
void ImportOPG();
void ImportFtsTxt();

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
void FtsTest();

void UpdateFtData();