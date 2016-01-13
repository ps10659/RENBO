#include "humanoid.h"
HANDLE sMhandle ;
HANDLE oBhandle[EVN_NUM] ;

int _tmain(int argc)
{
	WIN32_DAT *pWinData;
	RTN_ERR     ret;
	bool breakWhile = 0;
	int i = 0;


	printf("Wait...\n");

	ret = NEC_LoadRtxApp( "C:\\Program Files\\NEXCOM\\NexECMRtx\\Lib\\NexECMRtx\\x32\\NexECMRtx.rtss" );
	if( ret != 0 ){ printf("NEC_LoadRtxApp NexECMRtx.rtss failed!");}
	
	ret = NEC_LoadRtxApp( "C:..\\RTSSDebug\\RtxApp.rtss" ); 
	if( ret != 0 ){ printf( "NEC_LoadRtxApp() RtxApp.rtss error %d \n", ret );}
	
	ret = NEC_StartDriver();
	if( ret != 0 ) { printf("NEC_StarDriver failed!");}

	ret = NEC_GetRtMasterId( &masterId );
	if( ret != 0 ) { printf( "NEC_GetRtMasterId failed!" );}

	ret = NEC_ResetEcMaster( masterId );
	if( ret != 0 ) { printf( "NEC_ResetEcMaster failed!" );}

	ret = NEC_LoadNetworkConfig( masterId, "c:..\\..\\ENI\\ENI_33_Motor_hall.xml", START_NETWORK_OPT_MASK_NIC_PORT);
	if( ret != 0 ) { printf( "NEC_LoadNetworkConfig failed! (ENI_33_Motor.xml failed)" );}

	//system("pause");

	// open share memory
	sMhandle = RtOpenSharedMemory( SHM_MAP_ALL_ACCESS , 0 , SHM_NAME , &location );//開啟再RTX層creat的sharememory
	//SHM_MAP_ALL_ACCESS:可讀可寫
	//0:忽略
	//SHM_NAME:
	//&location:
	if( sMhandle == NULL )
	{
		printf("RtOpenSharedMemory fail");
		return 0;
	}
	pWinData = (WIN32_DAT *) location;
	pWinData->masterId = masterId;

	// open event
	for(i=0; i<EVN_NUM; i++)
	{
		oBhandle[i] = RtOpenEvent( NULL, 0, EVN_NAME[i] );//打開在RTX層creat的event
		if( oBhandle == NULL )
		{
			printf("RtOpenEvent %s fail\n", EVN_NAME[i]);
			return 0;
		}
	}

	// initital WinData
	InitPwindata(pWinData);

	pWinData->PP_totalPointCnt[0] = UserDefineTotalPoint; 
	pWinData->PP_totalPointCnt[1] = GoHomeTotalPoint; 
	pWinData->PP_totalPointCnt[2] = SquatTotalPoint; 
	pWinData->PP_totalPointCnt[3] = WalkingInitialReversePoint; 
	pWinData->PP_totalPointCnt[4] = WalkingInitialReversePoint; 
	pWinData->PP_totalPointCnt[5] = 1; 

	// update parameter
	ImportParameterTxt(pWinData);
	UpdateWalkingTrajectories(pWinData);

	printf("=======================\n");
	printf("WIN32_READY\n");
	printf("=======================\n\n");





	StartMaster(pWinData);
	SetMotorParam(pWinData);
	SetCurrPosHome(pWinData);
	HoldPos(pWinData);




	cout << "@@" << endl;
	system("pause");






	// keyboard input loop
	while(0)
	{
		while(pWinData->stateTransitionFlag){} // 這個應該沒用= =之後再修正

		if (_kbhit())
		{
			int key = _getch();

			if(key == '.')
			{
				UpdataAllActualTheta(pWinData);
				PrintAllActualTheta(pWinData);
				//PrintUserDefinedTheta();
			}
			else if(key == 'p')
			{
				ImportParameterTxt(pWinData);
			}
			else if(key == 'w')
			{
				WriteWalkingTrajectories(pWinData);
			}
			else if(key == '9')
			{
				WritePose(pWinData);
			}
			else if(key == '8')
			{
				//int actualPos=20;
				//*pActualPos=20;
				//NEC_RWProcessImage(pWinData->masterId, READ_PI, 781, (U8_T*)&actualPos, 4);
				//NEC_RWSlaveProcessImage(pWinData->masterId, 1025, READ_PI, 22, (U8_T*)&actualPos, 4);
				//printf("%d\n", actualPos);
				/*U8_T *pActualPos=new U8_T();
				*pActualPos=20;
				NEC_RWProcessImage(pWinData->masterId, READ_PDO, 871, pActualPos, 4);
				printf("%s\n", *pActualPos);*/
				//printf("%d\n", (I32_T)((pWinData->InputProcessDataPtr[784]*256*256*256 + pWinData->InputProcessDataPtr[783]*256*256 + pWinData->InputProcessDataPtr[782]*256 + pWinData->InputProcessDataPtr[781]) / axis_theta_to_motor_resolution[24] * 180.0/PI));
				//printf("%u %u\n", pWinData->InPDSizeInByte, pWinData->OutPDSizeInByte);
			}
			else if(key == 'g')
			{
				printf("\nContinue(1)\n");
				printf("Update walking trajectory(2)\n");
				while(1)
				{
					if (_kbhit())
					{
						int key = _getch();
						if(key == '1') break;
						else if(key == '2')
						{
							printf("\n   Updating...");
							UpdateWalkingTrajectories(pWinData);
							break;
						}
					}
				}

					pWinData->resetCntFlag = 1;
					pWinData->holdSwitch = 0;

					pWinData->currentState = CSP_RUN;

			}
			else
			{
				breakWhile = TriggerEvent(key, pWinData);
			}
			if(breakWhile) break;
		}
	}


	// close share memory
	RtCloseHandle(sMhandle);

	// close all event
	for(i=0; i<EVN_NUM; i++)
	{	
		RtCloseHandle(oBhandle[i]);
	}

	system("pause");
	return 0;
}



bool TriggerEvent(int key, WIN32_DAT *pWinData)
{

	switch (pWinData->currentState)
	{
		case BEGINNING:
			switch (key)
			{
				case '1':
					return goto_START_MASTER_AND_SLAVES(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case START_MASTER_AND_SLAVES:
			switch (key)
			{
				case '1':
					return goto_SET_MOTOR_PARAMETERS(pWinData);
				case '2':
					return goto_START_MASTER_AND_SLAVES(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case SET_MOTOR_PARAMETERS:
			switch (key)
			{
				case '1':
					return goto_SET_CURR_POS_HOME(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case SET_CURR_POS_HOME:
			switch (key)
			{
				case '1':
					return goto_HOLD(pWinData);
				case '2':
					return goto_SET_CURR_POS_HOME(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}
			
		case HOLD:
			switch (key)
			{
				case '0':
					return goto_HOMING_MODE(pWinData);
				case '1':
					return goto_PP_MODE(pWinData);
				case '2':
					return goto_CSP_MODE(pWinData);
				case 's':
					return goto_STOP_BUT_SOFT(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case HOMING:
			switch (key)
			{
				case '1':
					return goto_SET_CURR_POS_HOME(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case GO_HOME:
			switch (key)
			{
				case '1':
					return goto_SET_CURR_POS_HOME(pWinData);
				case 'h':
					return goto_STOP_AND_HOLD(pWinData);
				case 's':
					return goto_STOP_BUT_SOFT(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case STOP_AND_HOLD:
			switch (key)
			{
				case 's':
					return goto_STOP_BUT_SOFT(pWinData);
				case '0':
					return goto_GO_HOME(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case STOP_BUT_SOFT:
			switch (key)
			{
				case 'q':
					return goto_SERVO_OFF(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case SERVO_OFF:
			switch (key)
			{
				case '1':
					return goto_HOLD(pWinData);
				case '2':
					return goto_SET_CURR_POS_HOME(pWinData);
				case '3':
					return goto_WRITE_FILE(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case CLOSE_MASTER:
			switch (key)
			{
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}


		case HOMING_MODE:
			switch (key)
			{
				case '1':
					return goto_HOMING_RUN(pWinData);
				case 'h':
					return goto_HOLD(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case HOMING_RUN:
			switch (key)
			{
				/*case '1':
					return goto_HOMING_MODE(pWinData);*/
				case 'h':
					return goto_HOLD(pWinData);
				case 's':
					return goto_STOP_BUT_SOFT(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}
		case PP_MODE:
			switch (key)
			{
				case '1':
					return goto_PP_CHECK_IK_LIMIT(pWinData);
				case 'h':
					return goto_HOLD(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case PP_CHECK_IK_LIMIT:
			switch (key)
			{
				case '1':
					return goto_PP_RUN(pWinData);
				case '2':
					return goto_PP_MODE(pWinData);
				case 'h':
					return goto_HOLD(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case PP_RUN:
			switch (key)
			{
				case 'h':
					return goto_HOLD(pWinData);
				case 's':
					return goto_STOP_BUT_SOFT(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case CSP_MODE:
			switch (key)
			{
				case '1':
					return goto_CSP_CHECK_IK_LIMIT(pWinData);
				case 'h':
					return goto_HOLD(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case CSP_CHECK_IK_LIMIT:
			switch (key)
			{
				case '1':
					return goto_CSP_RUN(pWinData);
				case 'h':
					return goto_HOLD(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case CSP_RUN:
			switch (key)
			{
				case 'h':
					return goto_HOLD(pWinData);
				case 's':
					return goto_STOP_BUT_SOFT(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input.\n", key);
					return 0;
			}

		case WRITE_FILE:
			switch (key)
			{
				case '1':
					return goto_SET_CURR_POS_HOME(pWinData);
				case ESC_KEY:
					return goto_CLOSE_MASTER(pWinData);
				default:
					printf("%c is not a valid input ???.\n", key);
					return 0;
			}
	}
}

bool goto_START_MASTER_AND_SLAVES(WIN32_DAT *pWinData)
{			
	printf("\ncurrent state: START_MASTER_AND_SLAVES\n");
	printf("next state   : SET_MOTOR_PARAMETERS(1)\n");
	printf("               START_MASTER_AND_SLAVES(2)\n");

	pWinData->currentState = START_MASTER_AND_SLAVES;
	RtSetEvent(oBhandle[START_MASTER_AND_SLAVES]);
	return 0;
}
bool goto_SET_MOTOR_PARAMETERS(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: SET_MOTOR_PARAMETERS\n");
	printf("next state   : SET_CURR_POS_HOME(1)\n");
			
	pWinData->currentState = SET_MOTOR_PARAMETERS;
	RtSetEvent(oBhandle[SET_MOTOR_PARAMETERS]);
	return 0;
}
bool goto_SET_CURR_POS_HOME(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: SET_CURR_POS_HOME\n");
	printf("next state   : HOLD(1)\n");
	printf("               SET_CURR_POS_HOME(2)\n");

	//pWinData->home35CompleteFlag = 0;
	//while(!pWinData->home35CompleteFlag){}		


	pWinData->currentState = SET_CURR_POS_HOME;
	RtSetEvent(oBhandle[SET_CURR_POS_HOME]);
	return 0;
}
bool goto_HOMING(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: HOMING\n");
	printf("next state   : SET_CURR_POS_HOME(1)\n");		
	

	
	pWinData->currentState = HOMING;
	RtSetEvent(oBhandle[HOMING]);
	return 0;
}
bool goto_GO_HOME(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: GO_HOME\n");
	printf("next state   : SET_CURR_POS_HOME(1)\n");
	printf("               STOP_AND_HOLD(h)\n");
	printf("               STOP_BUT_SOFT(s)\n");
	
	pWinData->currentState = GO_HOME;
	RtSetEvent(oBhandle[GO_HOME]);
	return 0;
}
bool goto_STOP_AND_HOLD(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: STOP_AND_HOLD\n");
	printf("next state   : STOP_BUT_SOFT(s)\n");
	printf("               GO_HOME(0)\n");
			
	pWinData->currentState = STOP_AND_HOLD;
	pWinData->Flag_HoldPosSaved = 0;
	pWinData->holdSwitch = 1;
	RtSetEvent(oBhandle[STOP_AND_HOLD]);
	return 0;
}
bool goto_STOP_BUT_SOFT(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: STOP_BUT_SOFT\n");
	printf("next state   : SERVO_OFF(q)\n");
			
	pWinData->currentState = STOP_BUT_SOFT;
	pWinData->setTargetTorqueSwitch = 0;
	RtSetEvent(oBhandle[STOP_BUT_SOFT]);
	return 0;
}
bool goto_SERVO_OFF(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: SERVO_OFF\n");
	printf("next state   : Servo On and Hold(1)\n");
	printf("               SET_CURR_POS_HOME(2)\n");
	printf("               WRITE_FILE(3)\n");
	printf("               CLOSE_MASTER(esc)\n");

	pWinData->setServoOffFlag = 1;
	pWinData->setTargetTorqueSwitch = 0;
	pWinData->currentState = SERVO_OFF;
	RtSetEvent(oBhandle[SERVO_OFF]);
	return 0;
}
bool goto_CLOSE_MASTER(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: CLOSE_MASTER\n");

	pWinData->currentState = CLOSE_MASTER;
	RtSetEvent(oBhandle[CLOSE_MASTER]);
	return 1;
}

////////////////////////////////////////////////////////
bool goto_HOLD(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: HOLD\n");
	printf("next state   : HOMING_MODE(0)\n");
	printf("               PP_MODE(1)\n");
	printf("               CSP_MODE(2)\n");
	printf("               STOP_BUT_SOFT(s)\n");
		
	pWinData->Flag_HoldPosSaved = 0;
	pWinData->holdSwitch = 1;
	pWinData->setServoOnFlag = 1;
	pWinData->setTargetTorqueSwitch = 1;

	pWinData->currentState = HOLD;
	RtSetEvent(oBhandle[HOLD]);
	return 0;
}

////////////////////////////////////////////////////////
bool goto_HOMING_MODE(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: HOMING_MODE\n");
	printf("next state   : HOMING_RUN(1)\n");
	printf("               HOLD(h)\n");

	printf("\nCheck if the humanoid pose is close to home pose!!\n");

	pWinData->currentState = HOMING_MODE;
	RtSetEvent(oBhandle[HOMING_MODE]);
	return 0;
}
bool goto_HOMING_RUN(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: HOMING_RUN\n");
	printf("next state   : Set Current Position As Home(1)\n");
	printf("             : HOLD(h)\n");
	printf("               STOP_BUT_SOFT(s)\n");
	

	pWinData->currentState = HOMING_RUN;
	RtSetEvent(oBhandle[HOMING_RUN]);

	pWinData->HOMING_allHomeSensorReachFlag = 0;
	pWinData->resetCntFlag = 1;
	pWinData->holdSwitch = 0;


	int key;
	int splineType = 3;
	double goHomeTimePeriod = 3;
	int motionCnt = 0;

	while(1)
	{
		if( pWinData->HOMING_allHomeSensorReachFlag == 1 && motionCnt==0)
		{
			WritePose(pWinData);
			for(int i=0; i<TOTAL_AXIS; i++)
			{
				pWinData->PP_targetTheta[i] = pWinData->HOMING_homeSensorTheta[i] + pWinData->HOMING_homePositionOffset[i];
				//腳伸直->腳三點共線的角度差
				if(i==23 || i==29) pWinData->PP_targetTheta[i] += -15.65 * PI / 180.0;	//hip pitch
				else if(i==24 || i==30) pWinData->PP_targetTheta[i] += 34.08 * PI / 180.0;	//knee
				else if(i==25 || i==31) pWinData->PP_targetTheta[i] += -18.43 * PI / 180.0;	//ankld pitch
			}

			UpdateSplineVector(pWinData, splineType, goHomeTimePeriod);

			pWinData->currentState = PP_RUN;
			pWinData->PP_singleMovementCompleteFlag = 0;
			pWinData->resetCntFlag = 1;
			pWinData->holdSwitch = 0;
			motionCnt += 1;
		}

		if(_kbhit())
		{
			key = _getch();
			if(key == '1')
			{
				RtSetEvent(oBhandle[SET_CURR_POS_HOME]);
				break;
			}
			else if(key == 'h' || key == 's')
			{
				return goto_HOLD(pWinData);
				break;
			}
		}
	}

	

	return 0;
}

////////////////////////////////////////////////////////
bool goto_PP_MODE(WIN32_DAT *pWinData)
{

	printf("\nChoose \"Motion\" and \"Time Period(s)\": \n");
	printf("         User Input(0)\n");
	printf("         Go Home(1)\n");
	printf("         Squat(2)\n");
	printf("         Walking Initial(3)\n");
	printf("         XXXXXWalking Initial ReverseSquat(4)\n");
	printf("         ReadPoseTxt(5)\n");
	//printf("               Shake Hand(4)\n");
	cin >> pWinData->PP_motionType >> pWinData->PP_motionTimePeriod;
	pWinData->PP_currPointCnt = 0;

	if(pWinData->PP_motionType == 0) 
	{
		UpdataUserDefineData();
	}
	else if(pWinData->PP_motionType == 3) 
	{
		printf("\nContinue(1)\n");
		printf("Update walking trajectory(2)\n");

		while(1)
		{
			if (_kbhit())
			{
				int key = _getch();
				if(key == '1') break;
				else if(key == '2')
				{
					printf("\n   Updating...");
					UpdateWalkingTrajectories(pWinData);
					break;
				}
			}
		}
	}
	else if(pWinData->PP_motionType == 5) 
	{
		UpdateReadPoseTxtTheta(pWinData);
	}

	printf("\ncurrent state: PP_MODE\n");
	printf("next state   : PP_CHECK_IK_LIMIT(1)\n");
	printf("               HOLD(h)\n");

	pWinData->currentState = PP_MODE;
	RtSetEvent(oBhandle[PP_MODE]);
	return 0;
}
bool goto_PP_CHECK_IK_LIMIT(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: PP_CHECK_IK_LIMIT\n");
	printf("next state   : PP_RUN(1)\n");
	printf("               PP_MODE(2)\n");
	printf("               HOLD(h)\n");
	
	pWinData->currentState = PP_CHECK_IK_LIMIT;
	RtSetEvent(oBhandle[PP_CHECK_IK_LIMIT]);
	return 0;
}
bool goto_PP_RUN(WIN32_DAT *pWinData)
{
	int key;
	int splineType = 3;

	printf("\ncurrent state: PP_RUN\n");
	printf("next state: Hold(h)\n");
	printf("   \n");

	pWinData->currentState = PP_RUN;
	RtSetEvent(oBhandle[PP_RUN]);

	pWinData->PP_singleMovementCompleteFlag = 1; // initialize

	while(pWinData->PP_currPointCnt < pWinData->PP_totalPointCnt[pWinData->PP_motionType])
	{
		if(pWinData->PP_singleMovementCompleteFlag == 1)
		{
			SetPP_targetTheta(pWinData, pWinData->PP_motionType, pWinData->PP_currPointCnt);
			UpdateSplineVector(pWinData, splineType, pWinData->PP_motionTimePeriod);

			pWinData->PP_singleMovementCompleteFlag = 0;
			pWinData->resetCntFlag = 1;
			pWinData->holdSwitch = 0;
		}

		if(_kbhit())
		{
			key = _getch();
			if(key == 'h' || key == 's')
			{
				return goto_HOLD(pWinData);
				break;
			}
		}
	}

	
	return 0;
}

////////////////////////////////////////////////////////
bool goto_CSP_MODE(WIN32_DAT *pWinData)
{
	printf("\nContinue(1)\n");
	printf("Update walking trajectory(2)\n");
	while(1)
	{
		if (_kbhit())
		{
			int key = _getch();
			if(key == '1') break;
			else if(key == '2')
			{
				printf("\n   Updating...");
				UpdateWalkingTrajectories(pWinData);
				break;
			}
		}
	}

	printf("\ncurrent state: CSP_MODE\n");
	printf("next state   : CSP_CHECK_IK_LIMIT(1)\n");
	printf("               HOLD(h)\n");

	pWinData->currentState = CSP_MODE;
	return 0;
}
bool goto_CSP_CHECK_IK_LIMIT(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: CSP_CHECK_IK_LIMIT\n");
	printf("next state   : CSP_RUN(1)\n");
	printf("               HOLD(h)\n");
	
	pWinData->currentState = CSP_CHECK_IK_LIMIT;
	return 0;
}
bool goto_CSP_RUN(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: CSP_RUN\n");
	printf("next state   : STOP_AND_HOLD(h)\n");
	printf("n              STOP_BUT_SOFT(s)\n");
	
	pWinData->resetCntFlag = 1;
	pWinData->holdSwitch = 0;

	pWinData->currentState = CSP_RUN;
	return 0;
}

bool goto_WRITE_FILE(WIN32_DAT *pWinData)
{
	printf("\ncurrent state: WRITE_FILE\n");
	printf("next state   : SET_CURR_POS_HOME(1)\n");
	printf("               CLOSE_MASTER(esc)\n");

	pWinData->currentState = WRITE_FILE;
	RtSetEvent(oBhandle[WRITE_FILE]);
	return 0;
}


void UpdateSplineVector(WIN32_DAT *pWinData, int splineType, F64_T motionTimePeriod)
{
	int i, vec_length;
	Eigen::VectorXd spline_vec;
	vec_length = (int)(motionTimePeriod * SECOND_TO_MILISECOND / (F64_T)pWinData->cycleTime);
	pWinData->PP_motionTimeFrame = vec_length;
	//printf("\n length = %d\n\n", vec_length);
	spline_vec.resize(vec_length);

	for(i=0; i<MAX_MOTION_TIME_FRAME; i++)
	{
		pWinData->PP_splineVec[i] = 1;
	}

	if(splineType == 1){
		spline_vec.row(0).setLinSpaced(vec_length, 0, 1-1/vec_length);
		for(i=0; i<vec_length; i++)
		{
			pWinData->PP_splineVec[i] = spline_vec(i);
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
			pWinData->PP_splineVec[(int)j] = S(0) * pow(j,3) + S(1) * pow(j,2) + S(2) * j + S(3);
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
			pWinData->PP_splineVec[(int)j] = S(0) * pow(j,5) + S(1) * pow(j,4) + S(2) * pow(j,3) + S(3) * pow(j,2) + S(4) * j + S(5);
		}
	}
	//for(i=0; i<vec_length; i=i+300)
	//{
	//	printf("%f ", pWinData->PP_splineVec[(int)i]);
	//}
}
void UpdataAllActualTheta(WIN32_DAT *pWinData)
{
	pWinData->updateAllActualThetaFlag = 1;
	while(pWinData->updateAllActualThetaFlag){}
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
void PrintUserDefinedTheta()
{
	int i;
	printf("User defined theta:\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%f ", UserDefineTheta[0][i]);
		if(i%3 == 2) printf("\n");
	}
	printf("\n");
}
void UpdateAllKp(WIN32_DAT *pWinData)
{
	cout << "Motor, New Kp ratial, or -1 to leave\n";
	int index;
	double newKp;
	while(1)
	{
		cin >> index;
		if(index == -1) break;
		cin >> newKp;
		pWinData->Kp[index] = newKp;
	}
}
void UpdateAllKd(WIN32_DAT *pWinData)
{
	cout << "Motor, New Kd ratial, or -1 to leave\n";
	int index;
	double newKd;
	while(1)
	{
		cin >> index;
		if(index == -1) break;
		cin >> newKd;
		pWinData->Kd[index] = newKd;
	}
}
void PrintAllKp(WIN32_DAT *pWinData)
{
	int i;
	printf("All Kp:\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%d = %f, ", i, pWinData->Kp[i]);
		if(i%3 == 2) printf("\n");
	}
	printf("\n");
}
void PrintAllKd(WIN32_DAT *pWinData)
{
	int i;
	printf("All Kd:\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%d = %f, ", i, pWinData->Kd[i]);
		if(i%3 == 2) printf("\n");
	}
	printf("\n");
}

void SetPP_targetTheta(WIN32_DAT *pWinData, int motionType, int currPointCnt)
{
	int i;
	switch(motionType)
	{
		case 0:
			for(i=0; i<TOTAL_AXIS; i++)
			{
				pWinData->PP_targetTheta[i] = UserDefineTheta[currPointCnt][i]*PI/180.0;
			}
			break;
		case 1:
			for(i=0; i<TOTAL_AXIS; i++)
			{
				pWinData->PP_targetTheta[i] = HomeTheta[currPointCnt][i]*PI/180.0;
			}
			break;
		case 2:
			for(i=0; i<TOTAL_AXIS; i++)
			{
				pWinData->PP_targetTheta[i] = SquatTheta[currPointCnt][i]*PI/180.0;
			}
			break;
		case 3:
			for(i=0; i<TOTAL_AXIS; i++)
			{
				pWinData->PP_targetTheta[i] = WalkingInitialTheta[currPointCnt][i]*PI/180.0;
			}
			break;
		case 4:
			for(i=0; i<TOTAL_AXIS; i++)
			{
				pWinData->PP_targetTheta[i] = WalkingInitialReverseTheta[currPointCnt][i]*PI/180.0;
			}
			break;
		case 5:
			for(i=0; i<TOTAL_AXIS; i++)
			{
				pWinData->PP_targetTheta[i] = ReadPoseTxtTheta[currPointCnt][i]*PI/180.0;
			}
			break;
	}
}
void UpdataUserDefineData()
{
	int index;
	double degree;

	printf("==============================\n");
	printf("Motor targetTheta, or -1 to leave.\n");
	printf("==============================\n");

	while(1)
	{
		cin >> index;
		if(index == -1) break;
		cin >> degree;
		UserDefineTheta[0][index] = degree;
	}
}

void UpdateWalkingTrajectories(WIN32_DAT *pWinData)
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
		if(i<21)
		{
			WalkingInitialTheta[0][i] = 0;
		}
		else
		{
			WalkingInitialTheta[0][i] = (pWinData->WalkingTrajectories)[0][i] * 180 / PI;
			WalkingInitialReverseTheta[0][i] = -1 * (pWinData->WalkingTrajectories)[0][i] * 180 / PI;
		}
	}

	//for(i=21;i<TOTAL_AXIS;i++)
	//{
	//	printf("%f ", (pWinData->WalkingTrajectories)[0][i]);
	//}
	//printf("\n");

	fin.close();

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

void ImportParameterTxt(WIN32_DAT *pWinData)
{
	int i;
	ifstream fin;
	char buffer[20];
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
	for(i=0;i<TOTAL_AXIS;i++)
	{
		fin >> pWinData->HOMING_homePositionOffset[i];
		pWinData->HOMING_homePositionOffset[i] = -1.0 * pWinData->HOMING_homePositionOffset[i] * PI / 180.0;
	}

	printf("Import motor parameter\n");
	fin.close();

} 
void PrintImportParameterTxt(WIN32_DAT *pWinData)
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

	printf("HOMING_homePositionOffset\n");
	for(i=0; i<TOTAL_AXIS; i++)
	{
		printf("%f ", pWinData->HOMING_homePositionOffset[i]);
	}
	printf("\n\n");

}
void WritePose(WIN32_DAT *pWinData)
{
	fstream file;
	UpdataAllActualTheta(pWinData);

	file.open("C:..\\write_pose.txt", ios::app);
	for(int i = 0; i<33; i++)
		file << pWinData->actualTheta[i] * 180.0 / PI << " ";
	file << endl;

	printf("Write pose done\n");
	file.close();
}
void UpdateReadPoseTxtTheta(WIN32_DAT *pWinData)
{
	int i=0, j=0;
	ifstream fin;
	printf("Import replay_pose.txt\n");
    fin.open("C:..\\pose.txt", ios::in);

	fin >> pWinData->PP_totalPointCnt[5];
	printf("totalPointCnt = %d\n", pWinData->PP_totalPointCnt[5]);

	for(i=0;i<pWinData->PP_totalPointCnt[5];i++)
	{
		for(j=0; j<TOTAL_AXIS; j++)
		{
			fin >> ReadPoseTxtTheta[i][j];
		}
	}
	fin.close();
}

void InitPwindata(WIN32_DAT *pWinData)
{
	pWinData->Flag_StartMasterDone = 0;
	pWinData->Flag_SetMotorParameterDone = 0;
	pWinData->Flag_SetCurrPosHomeDone = 0;
}
void StartMaster(WIN32_DAT *pWinData)
{
	RtSetEvent(oBhandle[START_MASTER_AND_SLAVES]);
	while(!pWinData->Flag_StartMasterDone){}
	pWinData->Flag_StartMasterDone = 0;
}
void SetMotorParam(WIN32_DAT *pWinData)
{
	RtSetEvent(oBhandle[SET_MOTOR_PARAMETERS]);
	while(!pWinData->Flag_SetMotorParameterDone){}
	pWinData->Flag_SetMotorParameterDone = 0;
}
void SetCurrPosHome(WIN32_DAT *pWinData)
{
	RtSetEvent(oBhandle[SET_CURR_POS_HOME]);
	while(!pWinData->Flag_SetCurrPosHomeDone){}
	pWinData->Flag_SetCurrPosHomeDone = 0;
}
void HoldPos(WIN32_DAT *pWinData)
{
	pWinData->Flag_HoldPosSaved = 0;
	pWinData->holdSwitch = 1;
	pWinData->setServoOnFlag = 1;
	pWinData->setTargetTorqueSwitch = 1;
}