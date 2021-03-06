#include "humanoid.h"

HANDLE sMhandle ;
HANDLE oBhandle[EVN_NUM] ;

WIN32_DAT *pWinData;

int _tmain(int argc)
{
	RTN_ERR     ret;
	int i = 0;

if(1)
{
	/*system("RtssKill 002");
	system("RtssKill 001");*/

	cout << "loading NexECMRtx.rtss ..." << endl;
	ret = NEC_LoadRtxApp( "C:\\Program Files\\NEXCOM\\NexECMRtx\\Lib\\NexECMRtx\\x32\\NexECMRtx.rtss" );
	if( ret != 0 ){ printf("NEC_LoadRtxApp NexECMRtx.rtss failed!");}
	
	cout << "loading RxtApp.rtss ..." << endl;
	ret = NEC_LoadRtxApp( "C:..\\RTSSDebug\\RtxApp.rtss" ); 
	if( ret != 0 ){ printf( "NEC_LoadRtxApp() RtxApp.rtss error %d \n", ret );}
	
	ret = NEC_StartDriver();
	if( ret != 0 ) { printf("NEC_StarDriver failed!");}

	ret = NEC_GetRtMasterId( &masterId );
	if( ret != 0 ) { printf( "NEC_GetRtMasterId failed!" );}

	ret = NEC_ResetEcMaster( masterId );
	if( ret != 0 ) { printf( "NEC_ResetEcMaster failed!" );}

	cout << "loading ENI_33_Motor_hall.xml ..." << endl;
	ret = NEC_LoadNetworkConfig( masterId, "c:..\\..\\ENI\\ENI_33_Motor_hall.xml", START_NETWORK_OPT_MASK_NIC_PORT);
	if( ret != 0 ) { printf( "NEC_LoadNetworkConfig failed! (ENI_33_Motor.xml failed)" );}

	// open share memory created in RtxApp
	sMhandle = RtOpenSharedMemory( SHM_MAP_ALL_ACCESS , 0 , SHM_NAME , &location );
	if( sMhandle == NULL )
	{
		printf("RtOpenSharedMemory fail");
		return 0;
	}
	pWinData = (WIN32_DAT *) location;
	pWinData->masterId = masterId;

	// open event vreated in RtxApp
	for(i=0; i<EVN_NUM; i++)
	{
		oBhandle[i] = RtOpenEvent( NULL, 0, EVN_NAME[i] );
		if( oBhandle == NULL )
		{
			printf("RtOpenEvent %s fail\n", EVN_NAME[i]);
			return 0;
		}
	}

	// initital share memory pWinData
	InitPwindata(pWinData);


	// update parameter
	ImportParameterTxt();
	ImportOPG();
	ImportFtsTxt();
	UpdateWalkTraj();

	printf("=======================\n");
	printf("WIN32_READY\n");
	printf("=======================\n\n");


	StartMaster();
	SetMotorParam();
	SetCurrPosHome();
	HoldPos();
	//system("pause");
}

	// eigen test
	//Eigen::Vector2d a(1,2);
	//Eigen::Vector4d b;
	//b << a, Eigen::Vector2d(3,4);
	//cout << b << endl;
	//b << b(2), b(3), b(0), b(1);
	//cout << b << endl;
	//system("pause");
	
	InitForceSensor();
	DisplayOptions();
	
	while(1)
	{	
		if(_kbhit())
		{
			int key = _getch();
		
			switch(key)
			{
			case 's':
				cout << "soft" << endl;
				NoTorque();
				break;
			case 'h':
				cout << "hold" << endl;
				HoldPos();
				break;
			case 'o':
				cout << "pp: home" << endl;
				pWinData->Flag_ResetT_cog = 1;	
				PP_Move_deg(Pos_home, 2.0, false);
				break;
			case 'p':
			{
				cout << "pp: temp" << endl;
				UpdatePpPose();
				PP_Move_deg(Pos_temp, 2.0, false);
				break;
			}
			case 'P':
			{
				cout << "pp: choose" << endl;
				ListFiles(pose_file);
				int pose_num;
				cout << "choose pose: ";
				cin >> pose_num;
				UpdatePpPose(pose_num);
				PP_Move_deg(Pos_temp, 2.0, false);
				break;
			}
			case 'r':
			{
				cout << "csp: walking" << endl;
				ListFiles(walking_trajectory_file);
				int traj_num;
				cout << "choose trajectory: ";
				cin >> traj_num;
				UpdateWalkTraj(traj_num);
				
				PP_Move_rad(Pos_walking_inital, 3.0, true);
				system("pause");
				CSP_Run();
				break;
			}
			case 'R':
			{
				pWinData->Flag_PpReachTarget = 0;
				pWinData->target_cog[0] = 0;
				pWinData->target_cog[1] = 0;
				pWinData->target_cog[2] = pWinData->cog_height_for_IK;
				pWinData->target_left_foot[0] = 0;
				pWinData->target_left_foot[1] = 0.5 * pWinData->foot_distance;
				pWinData->target_left_foot[2] = 0;
				pWinData->target_right_foot[0] = 0;
				pWinData->target_right_foot[1] = -0.5 * pWinData->foot_distance;
				pWinData->target_right_foot[2] = 0;
				pWinData->Flag_ResetCnt = 1;
				
				pWinData->Flag_break_while = 0;
				pWinData->Flag_ResetStaticInOPG = 1;
				pWinData->next_state_cmd = 1;
				pWinData->curr_state = 1;
				pWinData->MotorState = MotorState_OPG;
				
				system("CLS");
				printf("[8] forward\n");
				printf("[5] stay\n");
				printf("[2] backward\n");
				printf("[0] stop walking\n");
				printf("[h] STOP immediately\n");

				while(pWinData->Flag_break_while != 1)
				{
					if(_kbhit())
					{
						int key = _getch();

						switch(key)
						{
						case '5':
							if(pWinData->next_state_cmd != -1 && pWinData->next_state_cmd != 0 && pWinData->next_state_cmd != 1)
								pWinData->next_state_cmd = 5;
							break;
						case '8':
							if(pWinData->next_state_cmd != -1 && pWinData->next_state_cmd != 0 && pWinData->next_state_cmd != 1)
							pWinData->next_state_cmd = 8;
							break;
						case '2':
							if(pWinData->next_state_cmd != -1 && pWinData->next_state_cmd != 0 && pWinData->next_state_cmd != 1)
							pWinData->next_state_cmd = 2;
							break;
						case '0':
							if(pWinData->curr_state == 5)
								pWinData->next_state_cmd = 0;
							break;
						case '9':
							ImportOPG();
							ImportFtsTxt();
							break;
						case '3':
							pWinData->global_cnt = 0;
							break;

						case 'h':
							pWinData->Flag_break_while = 1;
							HoldPos();
							break;
						}

						
					}
					system("CLS");

					UpdateFtData();

					cout << setw(10) << "zmp";
					cout << setw(10) << pWinData->zmp_x;
					cout << setw(10) << pWinData->zmp_y << endl << endl;

					cout << setw(10) << "zmp_left";
					cout << setw(10) << pWinData->zmp_lx;
					cout << setw(10) << pWinData->zmp_ly << endl << endl;

					cout << setw(10) << "zmp_right";
					cout << setw(10) << pWinData->zmp_rx;
					cout << setw(10) << pWinData->zmp_ry << endl << endl;

					cout << setw(10) << "fz_0/1";
					cout << setw(10) << pWinData->fz[0];
					cout << setw(10) << pWinData->fz[1] << endl << endl;

				}
				HoldPos();
				break;
			}
			case 't':
				cout << "fts test" << endl;
				FtsTest();
				break;

			case '7':
			{
				HOMING_MoveToHomeSensor();
				for(int i=0; i<TOTAL_AXIS; i++)
				{
					Pos_temp[i] = pWinData->HOMING_homeSensorTheta[i] + pWinData->HOMING_homePositionOffset[i];
					if(i==23 || i== 29) Pos_temp[i] += -15.65 * PI / 180.0;
					if(i==24 || i== 30) Pos_temp[i] += 34.08 * PI / 180.0;
					if(i==25 || i== 31) Pos_temp[i] += -18.43 * PI / 180.0;
				}
				PP_Move_rad(Pos_temp, 3.0, true);
				SetCurrPosHome();
				break;
			}
			case '8':
				cout << "set curr pos home" << endl;
				SetCurrPosHome();
				break;

			case '9':
				ImportParameterTxt();
				ImportOPG();
				ImportFtsTxt();
				break;

			case '3':
				WriteFtsData();
				break;
			case '2':
				ResetFtsOffset();
				break;

			case '.':
				WritePpPose();
				break;
			
			case ESC_KEY:
				goto _Byebye;
			}

		}
		UpdateFtData();
		DisplayOptions();
	}



_Byebye:

	HoldPos();
	ServoOff();
	CloseMaster();

	// close share memory
	RtCloseHandle(sMhandle);

	// close all event
	for(i=0; i<EVN_NUM; i++)
	{	
		RtCloseHandle(oBhandle[i]);
	}
	system("RtssKill 002");
	system("RtssKill 001");
	system("pause");
	return 0;
}


void DisplayOptions()
{
	system("CLS");

	printf("[s] soft\n");
	printf("[h] hold\n");
	printf("[o] Go to home pose\n");
	printf("[p] Go to temp pose\n");
	printf("[P] Go to selected pose\n");
	printf("[r] run selected walking traj");
	printf("[t] fts test\n\n");

	printf("[7] reset home position\n");
	printf("[8] set curr pos home\n");
	printf("[9] import parameter\n");
	printf("[3] write fts data\n");
	printf("[2] reset fts offset\n");
	printf("[.] Write pose \n");
	printf("[esc] Quit\n\n");

	cout << setw(10) << "F/T left";
	cout << setw(10) << pWinData->mx[0];
	cout << setw(10) << pWinData->my[0];
	cout << setw(10) << pWinData->mz[0];
	cout << setw(10) << pWinData->fx[0];
	cout << setw(10) << pWinData->fx[0];
	cout << setw(10) << pWinData->fz[0] << endl << endl;

	cout << setw(10) << "F/T right";
	cout << setw(10) << pWinData->mx[1];
	cout << setw(10) << pWinData->my[1];
	cout << setw(10) << pWinData->mz[1];
	cout << setw(10) << pWinData->fx[1];
	cout << setw(10) << pWinData->fx[1];
	cout << setw(10) << pWinData->fz[1] << endl << endl;

	cout << setw(10) << "zmp_left";
	cout << setw(10) << pWinData->zmp_lx;
	cout << setw(10) << pWinData->zmp_ly << endl << endl;

	cout << setw(10) << "zmp_left";
	cout << setw(10) << pWinData->zmp_rx;
	cout << setw(10) << pWinData->zmp_ry << endl << endl;

	//cout << setw(10) << pWinData->actual_cog[0];
	//cout << setw(10) << pWinData->actual_cog[1];
	//cout << setw(10) << pWinData->actual_cog[2] << endl;
	//cout << setw(10) << pWinData->actual_left_foot[0];
	//cout << setw(10) << pWinData->actual_left_foot[1];
	//cout << setw(10) << pWinData->actual_left_foot[2] << endl;
	//cout << setw(10) << pWinData->actual_right_foot[0];
	//cout << setw(10) << pWinData->actual_right_foot[1];
	//cout << setw(10) << pWinData->actual_right_foot[2] << endl << endl;

	cout << pWinData->Flag_PpReachTarget;
}

void UpdataAllActualTheta(WIN32_DAT *pWinData)
{
	pWinData->Flag_UpdateActualTheta = 1;
	while(pWinData->Flag_UpdateActualTheta){} // wait for the update 
}
void PrintAllActualTheta(WIN32_DAT *pWinData)
{
	int i;
	printf("Actual theta:\n");
	for(i=0; i<TOTAL_AXIS; i++) 
	{
		printf("%d = %f, ", i, pWinData->actualTheta[i] * 180.0 / PI );
		if(i%3 == 2) printf("\n");
	}
	printf("\n");
}


void printWalkingTrajectories(WIN32_DAT *pWinData)
{
	//int i;
	//for(i=21;i<TOTAL_AXIS;i++)
	//{
	//	printf("%f ", pWinData->WalkingTrajectories[0][i]);
	//	printf("%f ", pWinData->WalkingTrajectories[WALKING_TIMEFRAME-1][i]);
	//	printf("\n");
	//}
}
void WriteWalkingTrajectories(WIN32_DAT *pWinData)
{
	const int PrintCnt = 50;

	fstream file1;
	fstream file2;
	file1.open("C:..\\..\\WalkingTrajectories\\CommandWalkingTrajectories.txt", ios::out);
	file2.open("C:..\\..\\WalkingTrajectories\\ActualWalkingTrajectories.txt", ios::out);
	
	for(int i=21; i<33; i++)
	{
		for(int j=0; j<pWinData->walkingTimeframe; j+=PrintCnt)
		{
			file1 << (pWinData->WalkingTrajectories)[j][i] << " ";
			file2 << (pWinData->ActualWalkingTrajectories)[j][i] << " ";
		}
		file1<<"\n";
		file2<<"\n";
	}
	printf("Write walking trajectories done\n");

	file1.close();
	file2.close();
}
void WriteFtsData()
{
	fstream file;
	file.open("C:..\\..\\FtsData\\FtsData.txt", ios::out);
	
	for(int i=0; i<10000; i++)
	{
		file << pWinData->l_mx[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->l_my[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->l_fz[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->r_mx[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->r_my[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->r_fz[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->l_foot_x[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->l_foot_y[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->r_foot_x[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->r_foot_y[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->zmp0_x[i] << " ";
	}
	file<<"\n";
	
	for(int i=0; i<10000; i++)
	{
		file << pWinData->zmp0_y[i] << " ";
	}
	file<<"\n";

	for(int i=0; i<10000; i++)
	{
		file << pWinData->sup[i] << " ";
	}
	file<<"\n";
	
	file.close();
	printf("Write Fts and foot trajectories done\n");

}

void InitPwindata(WIN32_DAT *pWinData)
{
	pWinData->cycleTime = cycleTime;
	pWinData->MotorState = MotorState_NoTq;

	pWinData->Flag_StartMasterDone = 0;
	pWinData->Flag_SetMotorParameterDone = 0;
	pWinData->Flag_SetCurrPosHome = 0;

	pWinData->Flag_ServoOn = 0;
	pWinData->Flag_ServoOff = 0;
	pWinData->Flag_HoldPosSaved = 0;
	pWinData->Flag_UpdateActualTheta = 0;

	pWinData->PP_Queue_Rear = 0;
	pWinData->PP_Queue_Front = 0;
	pWinData->Flag_PpReachTarget = 1;
	pWinData->Flag_CspFinished = 0;
	pWinData->Flag_AllHomeSensorReached = 0;
	//pWinData->Flag_ResetT_cog = 1;	

	//GenerateCubicPolyVec(pWinData);
	UpdateSplineVec(pWinData->CubicPolyVec, MAX_MOTION_TIME_FRAME, 3);
	UpdateSplineVec(pWinData->leg_swing_xy_vec, 2500, 5);
	UpdateSwingVec(pWinData->leg_swing_z_vec, 5);
}
void GenerateCubicPolyVec(WIN32_DAT *pWinData)
{
	Eigen::VectorXd spline_vec;
	spline_vec.resize(MAX_MOTION_TIME_FRAME);

	double	x1 = 0,
			x2 = MAX_MOTION_TIME_FRAME,
			y1 = 0,
			ydot1 = 0,
			y2 = 1,
			ydot2 = 0;

	Eigen::MatrixXd A(4,4);
	A <<	pow(x1,3.0),		pow(x1,2.0),	x1,		1,
			3*pow(x1,2.0),		2*x1,			1,		0,
			pow(x2,3.0),		pow(x2,2.0),	x2,		1,
			3*pow(x2,2.0),		2*x2,			1,		0;

	Eigen::VectorXd B(4);
	B << y1, ydot1, y2, ydot2;

	Eigen::VectorXd S(4);
	S = A.inverse() * B;

	for(double i=0; i<MAX_MOTION_TIME_FRAME; i++){
		pWinData->CubicPolyVec[(int)i] = S(0) * pow(i,3) + S(1) * pow(i,2) + S(2) * i + S(3);
	}

	//for(int i=0; i<MAX_MOTION_TIME_FRAME; i+=500){
	//	cout << pWinData->CubicPolyVec[i];
	//}
}
void UpdateSplineVec(double *vec, int vec_length, int splineType)
{
	Eigen::VectorXd spline_vec;
	spline_vec.resize(vec_length);

	if(splineType == 1){
		spline_vec.row(0).setLinSpaced(vec_length, 0, 1-1/vec_length);
		for(int i=0; i<vec_length; i++)
		{
			vec[i] = spline_vec(i);
		}
	}
	else if(splineType == 3){
		double	x1 = 0,
				x2 = vec_length,
				y1 = 0,
				ydot1 = 0,
				y2 = 1,
				ydot2 = 0;

		Eigen::MatrixXd A(4,4);
		A << pow(x1,3.0),		pow(x1,2.0),	x1,		1,
			 3*pow(x1,2.0),		2*x1,			1,		0,
			 pow(x2,3.0),		pow(x2,2.0),	x2,		1,
			 3*pow(x2,2.0),		2*x2,			1,		0;

		Eigen::VectorXd B(4);
		B << y1, ydot1, y2, ydot2;

		Eigen::VectorXd S(4);
		S = A.inverse() * B;

		for(double j=0; j<vec_length; j++){
			//printf("%e\n", S(0) * pow(j,3) + S(1) * pow(j,2) + S(2) * j + S(3));
			vec[(int)j] = S(0) * pow(j,3) + S(1) * pow(j,2) + S(2) * j + S(3);
		}

	}
	else if(splineType == 5){
		double	x1 = 0,
				x2 = vec_length,
				y1 = 0,
				y1Vel = 0,
				y1Acc = 0,
				y2 = 1,
				y2Vel = 0,
				y2Acc = 0;
			
		Eigen::MatrixXd A(6,6);
		A << pow(x1,5),		pow(x1,4),		pow(x1,3),		pow(x1,2),	pow(x1,1),	1,
			 5*pow(x1,4),	4*pow(x1,3),	3*pow(x1,2),	2*x1,		1,			0,
			 20*pow(x1,3),	12*pow(x1,2),	6*x1,			2,			0,			0,
			 pow(x2,5),		pow(x2,4),		pow(x2,3),		pow(x2,2),	pow(x2,1),	1,
			 5*pow(x2,4),	4*pow(x2,3),	3*pow(x2,2),	2*x2,		1,			0,
			 20*pow(x2,3),	12*pow(x2,2),	6*x2,			2,			0,			0;

		Eigen::VectorXd B(6);
		B << y1, y1Vel, y1Acc, y2, y2Vel, y2Acc;

		Eigen::VectorXd S(6);
		S = A.inverse() * B;

		for(double j=0; j<vec_length; j++){
			vec[(int)j] = S(0) * pow(j,5) + S(1) * pow(j,4) + S(2) * pow(j,3) + S(3) * pow(j,2) + S(4) * j + S(5);
		}
	}
	//for(i=0; i<vec_length; i=i+300)
	//{
	//	printf("%f ", pWinData->PP_Spline_splineVec[(int)i]);
	//}
}
void UpdateSwingVec(double *vec, int splineType)
{
	// total 2500 time frames -> 2500*0.002 = 5 second
	double vec_1[1250];
	UpdateSplineVec(vec_1, 1250, splineType);
	for(int i=0; i<1250; i++){
		vec[i] = vec_1[i];
		vec[2499-i] = vec_1[i];
	}
}
void UpdateWalkTraj()
{
	int i=0, j=0, cnt=0;
	ifstream fin;
    fin.open("C:..\\..\\WalkingTrajectories\\WalkingTrajectories.txt", ios::in);

	fin>>pWinData->walkingTimeframe;
	printf("Update walking trajectories (total timeframe = %d)\n", pWinData->walkingTimeframe);
	if(pWinData->walkingTimeframe > MAX_WALKING_TIMEFRAME)
	{
		printf("walkingTimeframe > MAX_WALKING_TIMEFRAME\n");
	}

	for(i=0;i<TOTAL_AXIS;i++)
	{
		if(i==3 || i== 5 || i==8 || i==11 || i==19 || i==20 || i>=21)
		{
			for(j=0;j<pWinData->walkingTimeframe;j++)
			{
				if(!fin.eof())
				{
					fin>>(pWinData->WalkingTrajectories)[j][i];
					cnt++;
				}
			}
		}
		else
		{
			for(j=0;j<pWinData->walkingTimeframe;j++)
			{ 
				pWinData->WalkingTrajectories[j][i] = 0;
			}
		}
	}

	// update walking initial theta
	
	for(i=0;i<TOTAL_AXIS;i++)
	{
		Pos_walking_inital[i] = (pWinData->WalkingTrajectories)[0][i];
	}

	//for(i=21;i<TOTAL_AXIS;i++)
	//{
	//	printf("%f ", (pWinData->WalkingTrajectories)[0][i]);
	//}
	//printf("\n");

	fin.close();

}
void UpdateWalkTraj(int traj_num)
{
	int i=0, j=0, cnt=0;
	ifstream fin;
	string temp = walking_traj_dir;
	string walking_traj_file = temp.append(file_list[traj_num]);
    fin.open(walking_traj_file, ios::in);
	cout << walking_traj_file << endl;

	fin>>pWinData->walkingTimeframe;	
	printf("Update walking trajectories (total timeframe = %d)\n", pWinData->walkingTimeframe);
	if(pWinData->walkingTimeframe > MAX_WALKING_TIMEFRAME)
	{
		printf("walkingTimeframe > MAX_WALKING_TIMEFRAME\n");
	}
	for(i=0;i<TOTAL_AXIS;i++)
	{
		if(i==3 || i== 5 || i==8 || i==11 || i==19 || i==20 || i>=21)
		{
			for(j=0;j<pWinData->walkingTimeframe;j++)
			{
				if(!fin.eof())
				{
					fin>>(pWinData->WalkingTrajectories)[j][i];
					cnt++;
				}
			}
		}
		else
		{
			for(j=0;j<pWinData->walkingTimeframe;j++)
			{ 
				pWinData->WalkingTrajectories[j][i] = 0;
			}
		}
	}

	// update walking initial theta
	
	for(i=0;i<TOTAL_AXIS;i++)
	{
		Pos_walking_inital[i] = (pWinData->WalkingTrajectories)[0][i];
	}

	//for(i=21;i<TOTAL_AXIS;i++)
	//{
	//	printf("%f ", (pWinData->WalkingTrajectories)[0][i]);
	//}
	//printf("\n");

	fin.close();
}
void UpdatePpPose()
{
	int i=0;
	ifstream fin;
    fin.open("C:..\\..\\Poses\\temp.txt", ios::in);

	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin>>Pos_temp[i];
	}

	fin.close();
}
void UpdatePpPose(int pose_num)
{
	int i=0;
	ifstream fin;
	string temp = pose_dir;
	string pose_file = temp.append(file_list[pose_num]);
    fin.open(pose_file, ios::in);
	cout << pose_file << endl;

	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin>>Pos_temp[i];
	}

	fin.close();
}
void WritePpPose()
{
	string file_name;
	cout << "	file name: ";
	cin >> file_name;

	fstream file;
	UpdataAllActualTheta(pWinData);

	file.open(pose_dir + file_name, ios::app);
	for(int i = 0; i<33; i++)
		file << pWinData->actualTheta[i] * 180.0 / PI << " ";
	file << endl;

	printf("	Write pose done\n");
	file.close();
}
void ImportParameterTxt()
{
	int i;
	ifstream fin;
	char buffer[40];
    fin.open("C:..\\parameter.txt", ios::in);

	fin >> buffer;
	fin >> pWinData->walkingSpeed;

	fin >> buffer;
	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin >> pWinData->motorTorqueSwitch[i];
	}
	
	fin >> buffer;
	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin >> pWinData->Kp[i];
	}

	fin >> buffer;
	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin >> pWinData->Ki[i];
	}

	fin >> buffer;
	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin >> pWinData->Kd[i];
	}

	fin >> buffer;
	fin >> pWinData->Fts_LRK;
	fin >> pWinData->Fts_LRC;
	fin >> pWinData->Fts_LPK;
	fin >> pWinData->Fts_LPC;
	fin >> pWinData->Fts_RRK;
	fin >> pWinData->Fts_RRC;
	fin >> pWinData->Fts_RPK;
	fin >> pWinData->Fts_RPC;

	fin >> buffer;
	fin >> pWinData->FzThreshold;
	fin >> pWinData->MxyThreshold;

	fin >> buffer;
	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin >> pWinData->HOMING_homePositionOffset[i];
		pWinData->HOMING_homePositionOffset[i] = -1.0 * pWinData->HOMING_homePositionOffset[i] * PI / 180.0;
	}

	fin >> buffer;
	fin >> pWinData->S_Step;
	fin >> pWinData->adaptive_cnt;

	printf("Import motor parameter\n");
	//PrintImportParameterTxt();
	Sleep(500);
	fin.close();

} 
void PrintImportParameterTxt()
{
	int i;
	printf("walkingSpeed\n");
	printf("%f\n", pWinData->walkingSpeed);

	printf("motorTorqueSwitch\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%f ", pWinData->motorTorqueSwitch[i]);
	}
	printf("\n");

	printf("Kp\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%f ", pWinData->Kp[i]);
	}
	printf("\n");

	printf("Kd\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%f ", pWinData->Kd[i]);
	}
	printf("\n");

	printf("LRK,LRC,LPK,LPC,RRK,RRC,RPK,RPC\n");
	printf("%f %f %f %f %f %f %f %f\n", pWinData->Fts_LRK, pWinData->Fts_LRC, pWinData->Fts_LPK, pWinData->Fts_LPC, pWinData->Fts_RRK, pWinData->Fts_RRC, pWinData->Fts_RPK, pWinData->Fts_RPC);

	printf("FzThreshold,MxyThreshold\n");
	printf("%f %f\n", pWinData->FzThreshold, pWinData->MxyThreshold);

	printf("S_Step, adaptive_cnt\n");
	printf("%f\n", pWinData->S_Step);
	printf("%f\n", pWinData->adaptive_cnt);
	/*printf("HOMING_homePositionOffset\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%f ", pWinData->HOMING_homePositionOffset[i]);
	}*/
	printf("\n\n");


}
void ImportOPG()
{
	int i;

	ifstream fin;
	char buffer[40];
    fin.open("C:..\\OPG.txt", ios::in);

	fin >> buffer;
	fin >> pWinData->step_time;

	fin >> buffer;
	fin >> pWinData->cog_height_for_omega;

	fin >> buffer;
	fin >> pWinData->cog_height_for_IK;

	fin >> buffer;
	fin >> pWinData->foot_distance;

	fin >> buffer;
	fin >> pWinData->step_length;

	fin >> buffer;
	fin >> pWinData->swing_leg_height;

	fin >> buffer;
	fin >> i;
	UpdateSplineVec(pWinData->leg_swing_xy_vec, 2500, i);

	fin >> buffer;
	fin >> i;
	UpdateSwingVec(pWinData->leg_swing_z_vec, i);

	fin >> buffer;
	fin >> pWinData->gc_l_hip_pitch;
	
	fin >> buffer;
	fin >> pWinData->gc_r_hip_pitch;
	
	fin >> buffer;
	fin >> pWinData->gc_l_ankle_pitch;
	
	fin >> buffer;
	fin >> pWinData->gc_l_ankle_pitch;
}
void ImportFtsTxt()
{
	ifstream fin;
	char buffer[40];
    fin.open("C:..\\fts.txt", ios::in);
	
	fin >> buffer;
	fin >> pWinData->zmp_unit;

	fin >> buffer;
	fin >> pWinData->mx_offset[0];

	fin >> buffer;
	fin >> pWinData->mx_offset[1];

	fin >> buffer;
	fin >> pWinData->my_offset[0];

	fin >> buffer;
	fin >> pWinData->my_offset[1];

	fin >> buffer;
	fin >> pWinData->fz_offset[0];

	fin >> buffer;
	fin >> pWinData->fz_offset[1];
}

void StartMaster()
{
	RtSetEvent(oBhandle[START_MASTER_AND_SLAVES]);
	while(!pWinData->Flag_StartMasterDone){}
	pWinData->Flag_StartMasterDone = 0;
}
void SetMotorParam()
{
	RtSetEvent(oBhandle[SET_MOTOR_PARAMETERS]);
	while(!pWinData->Flag_SetMotorParameterDone){}
	pWinData->Flag_SetMotorParameterDone = 0;
}
void SetCurrPosHome()
{
	/*RtSetEvent(oBhandle[SET_CURR_POS_HOME]);
	while(!pWinData->Flag_SetCurrPosHomeDone){}
	pWinData->Flag_SetCurrPosHomeDone = 0;*/
	pWinData->Flag_SetCurrPosHome = 1;
}
void ServoOff()
{
	pWinData->Flag_ServoOff = 1;
}
void CloseMaster()
{
	RtSetEvent(oBhandle[CLOSE_MASTER]);
}

void NoTorque()
{
	pWinData->MotorState = MotorState_NoTq;
}
void HoldPos()
{
	// reset
	pWinData->PP_Queue_Rear = pWinData->PP_Queue_Front;	// clear PP_Queue
	pWinData->Flag_PpReachTarget = 1;	
	pWinData->Flag_CspFinished = 0;
	pWinData->Flag_ResetCbErrorTheta = 1;

	pWinData->Flag_HoldPosSaved = 0;
	pWinData->MotorState = MotorState_Hold;
	pWinData->Flag_ServoOn = 1;
}
void HOMING_MoveToHomeSensor()
{
	if(pWinData->Flag_AllHomeSensorReached == 1)
	{
		pWinData->Flag_AllHomeSensorReached = 0;
	}
	pWinData->Flag_ResetCnt;
	pWinData->MotorState = MotorState_Homing;

	while(pWinData->Flag_AllHomeSensorReached == 0){};
}
void PP_Move_rad(double *PP_targetTheta, double timePeriod, bool wait_until_reach)
{
	if((pWinData->PP_Queue_Front - pWinData->PP_Queue_Rear) % PP_QUEUE_SIZE == 1) // PP_Queue is full
	{
		cout << "PP_Queue is full" << endl;
		return;
	}
	else // push object into PP_Queue
	{
		pWinData->PP_Queue_Rear = (pWinData->PP_Queue_Rear + 1) % PP_QUEUE_SIZE;

		cout << pWinData->PP_Queue_Front << endl;
		cout << pWinData->PP_Queue_Rear << endl;

		pWinData->PP_Queue_TimePeriod[pWinData->PP_Queue_Rear] = timePeriod;
		for(int i=0; i<TOTAL_AXIS; i++)
		{
			pWinData->PP_Queue_TargetTheta[pWinData->PP_Queue_Rear][i] = PP_targetTheta[i];
		}
	}

	pWinData->MotorState = MotorState_PP;

	if(wait_until_reach)
	{
		while((pWinData->PP_Queue_Rear != pWinData->PP_Queue_Front) || !pWinData->Flag_PpReachTarget){}
	}
}
void PP_Move_deg(double *PP_targetDeg, double timePeriod, bool wait_until_reach)
{
	if((pWinData->PP_Queue_Front - pWinData->PP_Queue_Rear) % PP_QUEUE_SIZE == 1) // PP_Queue is full
	{
		cout << "PP_Queue is full" << endl;
		return;
	}
	else // push object into PP_Queue
	{
		pWinData->PP_Queue_Rear = (pWinData->PP_Queue_Rear + 1) % PP_QUEUE_SIZE;
		pWinData->PP_Queue_TimePeriod[pWinData->PP_Queue_Rear] = timePeriod;
		for(int i=0; i<TOTAL_AXIS; i++)
		{
			pWinData->PP_Queue_TargetTheta[pWinData->PP_Queue_Rear][i] = PP_targetDeg[i] * PI / 180.0;
		}
	}

	pWinData->MotorState = MotorState_PP;
	
	if(wait_until_reach == true)
	{
		while((pWinData->PP_Queue_Rear != pWinData->PP_Queue_Front) || !pWinData->Flag_PpReachTarget){}
	}
}
void CSP_Run()
{
	if(pWinData->Flag_CspFinished == 1)
	{
		pWinData->Flag_CspFinished = 0;
	}
	pWinData->Flag_ResetCnt;
	pWinData->MotorState = MotorState_CSP;
}
void FtsTest()
{
	pWinData->Flag_ResetCnt = 1;
	pWinData->MotorState = MotorState_FtsTest;
}
void UpdateFtData()
{
	double fz_threshold = 100;
	double l_weighting;
	getForceData(&fts);

	pWinData->mx[0] = fts.mx[0] + pWinData->mx_offset[0];
	pWinData->my[0] = fts.my[0] + pWinData->my_offset[0];
	pWinData->mz[0] = fts.mz[0] * -1+ pWinData->mz_offset[0];
	pWinData->fx[0] = fts.fx[0];
	pWinData->fy[0] = fts.fy[0];
	pWinData->fz[0] = fts.fz[0] * -1 + pWinData->fz_offset[0];

	pWinData->mx[1] = fts.mx[1] + pWinData->mx_offset[1];;
	pWinData->my[1] = fts.my[1] + pWinData->my_offset[1];;
	pWinData->mz[1] = fts.mz[1] * -1 + pWinData->mz_offset[1];
	pWinData->fx[1] = fts.fx[1];
	pWinData->fy[1] = fts.fy[1];
	pWinData->fz[1] = fts.fz[1] * -1 + pWinData->fz_offset[1];


	if(pWinData->fz[0] > fz_threshold && pWinData->fz[1] > fz_threshold)
	{
		pWinData->zmp_lx = -1 * pWinData->my[0] / pWinData->fz[0];
		pWinData->zmp_ly = pWinData->mx[0] / pWinData->fz[0];

		pWinData->zmp_rx = -1 * pWinData->my[1] / pWinData->fz[1];
		pWinData->zmp_ry = pWinData->mx[1] / pWinData->fz[1];

		l_weighting = pWinData->fz[0] / (pWinData->fz[0] + pWinData->fz[1]);

		pWinData->zmp_x = l_weighting * (pWinData->target_left_foot[0] + pWinData->zmp_lx) + (1 - l_weighting) * (pWinData->target_right_foot[0] + pWinData->zmp_rx);
		pWinData->zmp_y = l_weighting * (pWinData->target_left_foot[1] + pWinData->zmp_ly) + (1 - l_weighting) * (pWinData->target_right_foot[1] + pWinData->zmp_ry);
	}
	else if(pWinData->fz[0] > fz_threshold)
	{
		pWinData->zmp_lx = -1 * pWinData->my[0] / pWinData->fz[0];
		pWinData->zmp_ly = pWinData->mx[0] / pWinData->fz[0];

		pWinData->zmp_x = pWinData->target_left_foot[0] + pWinData->zmp_lx;
		pWinData->zmp_y = pWinData->target_left_foot[1] + pWinData->zmp_ly;
	}
	else if(pWinData->fz[1] > fz_threshold)
	{
		pWinData->zmp_rx = -1 * pWinData->my[1] / pWinData->fz[1];
		pWinData->zmp_ry = pWinData->mx[1] / pWinData->fz[1];

		pWinData->zmp_x = pWinData->target_right_foot[0] + pWinData->zmp_rx;
		pWinData->zmp_y = pWinData->target_right_foot[1] + pWinData->zmp_ry;
	}
	else
	{
		pWinData->zmp_x = 0;
		pWinData->zmp_y = 0;
	}

	pWinData->zmp_lx *= pWinData->zmp_unit;
	pWinData->zmp_ly *= pWinData->zmp_unit;
	pWinData->zmp_rx *= pWinData->zmp_unit;
	pWinData->zmp_ry *= pWinData->zmp_unit;
	pWinData->zmp_x *= pWinData->zmp_unit;
	pWinData->zmp_y *= pWinData->zmp_unit;

}
void ResetFtsOffset()
{
	int num = 100;

	double mx_sum[2] = {0};
	double my_sum[2] = {0};
	double mz_sum[2] = {0};
	double fz_sum[2] = {0};


	for(int i=0; i<num; i++)
	{
		UpdateFtData();
		for(int j=0; j<2; j++)
		{
			mx_sum[j] += pWinData->mx[j];
			my_sum[j] += pWinData->my[j];
			mz_sum[j] += pWinData->mz[j];
			fz_sum[j] += pWinData->fz[j];
		}
	}

	for(int j=0; j<2; j++)
	{
		pWinData->mx_offset[j] += -mx_sum[j]/num;
		pWinData->my_offset[j] += -my_sum[j]/num;
		pWinData->mz_offset[j] += -mz_sum[j]/num;
		pWinData->fz_offset[j] += -fz_sum[j]/num;
	}
}