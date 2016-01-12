#include "RtxApp.h"



void 
_cdecl
main(
       int  argc,
       char **argv,
       char **envp
    )
{
	USER_DAT	*pData;
	HANDLE		sHhandle;
	HANDLE		oBhandle[EVN_NUM];
	int			i;


	sHhandle = RtCreateSharedMemory ((DWORD) PAGE_READWRITE , (DWORD) 0 , (DWORD) sizeof(USER_DAT) , SHM_NAME , &location);
	//PAGE_READWRITE:Gives read-write access to the committed region of pages.
	//0:不需要開MaximumSizeHigh
	//sizeof(smemory):MaximumSizeLow的大小
	//SHM_NAME:那塊sharememory的名稱
	//&location:那塊Sharememory的位址
	if( sHhandle == NULL )
	{
		RtPrintf("Create RtCreateSharedMemory fail\n");
		ExitProcess(0);
	}
	pData = (USER_DAT *) (location);//將location強制轉型成char,由pData取代
	

	for(i=0; i<EVN_NUM; i++)
	{
		oBhandle[i] = RtCreateEvent( NULL, 0, FALSE, EVN_NAME[i] );
		//NULL:忽略
		//0: auto-reset event
		//FALSE:The initial state of the event object. If TRUE, the initial state is signaled; otherwise, it is non-signaled.
		//EVN_NAME:SHM_NAME:那塊sharememory的名稱
		if( oBhandle == NULL )
		{
			RtPrintf("Create RtCreateEvent %s fail\n", EVN_NAME[i]);
			ExitProcess(0);
		}
	}

	// init global variables
	for( i = 0; i<TOTAL_AXIS; ++i )
	{
        axis_theta_to_motor_resolution[i] = motor_direction[i] * reduction_ratio[i] * resolution[i] / (2.0*PI);
	}

	// init USER_DAT
	pData->cycleTime = cycleTime;
	pData->currentState = BEGINNING;
	pData->stateTransitionFlag = 0;
	pData->setServoOnFlag = 0;
	pData->setServoOffFlag = 0;
	pData->setTargetTorqueSwitch = 0;
	pData->firstTimeHoldFlag = 0;
	//pData->PP_allMovementCompleteFlag = 1;
	//pData->PP_singleMovementCompleteFlag = 1;
	pData->PP_currPointCnt = 0;

	for(i=0; i<MAX_MOTION_TIME_FRAME; i++)
	{
		pData->PP_splineVec[i] = 0;
	}


	RtPrintf("===========================\n");
	RtPrintf("RTX_READY\n");
	RtPrintf("===========================\n\n");

	while(1)
	{

		ret = RtWaitForSingleObject( oBhandle[START_MASTER_AND_SLAVES], 5 );
		//oBhandle:要等待的觸發事件
		//>0: milisecond, -1:waiting forever
		if( ret == 0 )
		{
			RtPrintf("START_MASTER_AND_SLAVES...\n");
			pData->stateTransitionFlag = 1;
			StartMaster(pData);
			pData->stateTransitionFlag = 0;
			RtPrintf("done\n\n");
		}

		ret = RtWaitForSingleObject( oBhandle[SET_MOTOR_PARAMETERS], 5 );
		if( ret == 0 )
		{
			RtPrintf( "SET_MOTOR_PARAMETERS...\n" );
			SetMotorParameters(pData);
			RtPrintf("done\n\n");
		}

		ret = RtWaitForSingleObject( oBhandle[SERVO_ON_AND_SET_CURR_POS_AS_HOME], 5 ); //?????????????????????????????????????????????????
		if( ret == 0 )
		{
			RtPrintf( "SERVO_ON_AND_SET_CURR_POS_AS_HOME...\n" );
			HomingMethod35(pData);
			//pData->home35CompleteFlag = 1;
			RtPrintf("done\n\n");
		}

		ret = RtWaitForSingleObject( oBhandle[CLOSE_MASTER], 5 );
		if( ret == 0 )
		{
			RtPrintf( "CLOSE_MASTER...\n" );
			goto _ERR;
		}

	}


_ERR:
// Close process...

	RtCloseHandle(sHhandle);//remember closed what you creat....

	for(i=0; i<EVN_NUM; i++)
	{
		RtCloseHandle(oBhandle[i]);//remember closed what you creat....
	}

	CloseMaster(pData);

	ExitProcess(0);

}

void __RtCyclicCallback( void *UserDataPtr )
{
	#define PRINT_COUNT 200

	USER_DAT		*pData = (USER_DAT *)UserDataPtr;
	U16_T			state;
	int				k = 0;

	// counter
	static I32_T	cnt = 0; // global counter
	static I32_T	HOMING_cnt = 0;
	static I32_T	PP_cnt = 0;
	static I32_T	CSP_cnt = 0;

	// state
	static F64_T	CB_targetTheta[TOTAL_AXIS];
	I32_T			actualEncoderPos[TOTAL_AXIS];
	F64_T			actualTheta[TOTAL_AXIS];
	F64_T			errorTheta[TOTAL_AXIS];
	F64_T			errorThetaDot[TOTAL_AXIS];
	static F64_T	preErrorTheta[TOTAL_AXIS];
	static F64_T	errorThetaSum[TOTAL_AXIS];
	F64_T			targetTorque[TOTAL_AXIS];
	I16_T			trimmedTargetTorque[TOTAL_AXIS];

	// homing related
	I32_T			homeSensorValue[TOTAL_AXIS];



	// initialization
	if(cnt==0)
	{
		for(k=0; k<TOTAL_AXIS; k++){
			actualEncoderPos[k] = 0;
			CB_targetTheta[k] = 0;
			//CB_targetPosition[k] = 0;
			preErrorTheta[k] = 0;
			errorThetaSum[k] = 0;
			pData->PP_singleMovementCompleteFlag = 1;
		}
	}

	NEC_RtGetMasterState( pData->masterId, &state );
	switch( state )
	{
	case ECM_STA_OPERATION:
		NEC_CoE402CyclicProcess();
		
		// variabls needed to be updated every cycle
		for(k=0; k<TOTAL_AXIS; k++) 
		{
			NEC_CoE402GetActualPosition( pData->axis[k], &actualEncoderPos[k]);
			actualTheta[k] = ((F64_T)actualEncoderPos[k]) / axis_theta_to_motor_resolution[k];	
		}

		// flags
		if(pData->setServoOnFlag == 1)
		{
			for(k=0; k<TOTAL_AXIS; k++) 
			{
				NEC_CoE402SetState( pData->axis[k], 3);
			}
			pData->setServoOnFlag = 0;
		}
		if(pData->setServoOffFlag == 1)
		{
			for(k=0; k<TOTAL_AXIS; k++) 
			{
				NEC_CoE402SetState( pData->axis[k], 0);
			}
			pData->setServoOffFlag = 0;
		}
		if(pData->updateAllActualThetaFlag == 1)
		{
			for(k=0; k<TOTAL_AXIS; k++) 
			{
				pData->actualTheta[k] = actualTheta[k];
			}
			pData->updateAllActualThetaFlag = 0;
		}
		if(pData->resetCntFlag == 1)
		{
			HOMING_cnt = 0;
			CSP_cnt = 0;
			PP_cnt = 0;
			pData->resetCntFlag = 0;
		}


		// [IMPORTANT FLAG] decide whether give the target torque or zero torque
		if( pData->setTargetTorqueSwitch )
		{

			// hold mode, set CB_targetTheta = actualTheta
			if( pData->holdSwitch )
			{
				if(pData->firstTimeHoldFlag == 0)
				{	
					for(k=0; k<TOTAL_AXIS; k++) 
					{	
						errorThetaSum[k] = 0;
						CB_targetTheta[k] = actualTheta[k];
					}

					pData->firstTimeHoldFlag = 1;
				}
			}
			else
			{
				//op mode, set CB_targetTheta depend on operation mode
				switch(pData->currentState)
				{
					case HOMING_RUN:
						HOMING_UpdateCbTargetTheta(CB_targetTheta, pData, actualTheta, homeSensorValue, &HOMING_cnt);
						if(pData->HOMING_allHomeSensorReachFlag == 1)
						{
							pData->resetCntFlag = 1;
							pData->firstTimeHoldFlag = 0;
							pData->holdSwitch = 1;
						}
						break;
					case PP_RUN:
						PP_UpdateCbTargetTheta(CB_targetTheta, pData, actualTheta, &PP_cnt);
						if(PP_cnt == pData->PP_motionTimeFrame) 
						{
							pData->PP_singleMovementCompleteFlag = 1;
							pData->PP_currPointCnt++;
							pData->resetCntFlag = 1;
							pData->firstTimeHoldFlag = 0;
							pData->holdSwitch = 1;
						}
						break;
					case CSP_RUN:
						CSP_UpdateCbTargetTheta(CB_targetTheta, pData, actualTheta, &CSP_cnt);
						if(CSP_cnt >= pData->walkingTimeframe / pData->walkingSpeed) 
						{
							pData->resetCntFlag = 1;
							pData->firstTimeHoldFlag = 0;
							pData->holdSwitch = 1;
						}
						break;
				}

				// TODO
				// safty guard
				// if CB_targetTheta > limit, hold, errorThetaSum=0
				// if CB_targetTheta - actualTheta > threshold, hold, errorThetaSum=0

			}

			// compute and output targetTorque here
			for(k=0; k<TOTAL_AXIS; k++)
			{
				errorTheta[k] = CB_targetTheta[k] - actualTheta[k];
				if(errorThetaSum[k] < maxErrorThetaSum)
					errorThetaSum[k] = errorThetaSum[k] + errorTheta[k];
				errorThetaDot[k] = (errorTheta[k] - preErrorTheta[k]) / (((F64_T)pData->cycleTime) * MILISECOND_TO_SECOND);
				preErrorTheta[k] = errorTheta[k];
				targetTorque[k] = (pData->Kp[k] * errorTheta[k] + pData->Ki[k] * errorThetaSum[k] + pData->Kd[k] * errorThetaDot[k]) * motor_direction[k];
				
				if(pData->motorTorqueSwitch[k] == 0) targetTorque[k] = 0;
				trimmedTargetTorque[k] = TargetTorqueTrimming( targetTorque[k] );

				// !!
				NEC_CoE402SetTargetTorque( pData->axis[k], trimmedTargetTorque[k] );
				// !!
				
			}
		}
		else
		{
			for(k=0; k<TOTAL_AXIS; k++)
			{
				errorThetaSum[k] = 0;

				// !!
				NEC_CoE402SetTargetTorque( pData->axis[k], 0 );
				// !!
			}
		}
		cnt++;

		
		//if( ( cnt % PRINT_COUNT ) == 0 )
		//{
		//	k=27;
		//	//NEC_RtGetProcessDataInput(pData->masterId, 679, 4, (U8_T*)&homeSensorValue[k]);
		//	NEC_RtGetSlaveProcessDataInput(pData->masterId, k, 10, (U8_T*)&homeSensorValue[k], 4);
		//	RtPrintf("       %d, targetTheta %d, actualTheta %d, actualTorque %d, test: %d\n", 
		//		k, 
		//		(I32_T)(CB_targetTheta[k]*180.0/PI),
		//		(I32_T)(actualTheta[k]*180.0/PI), 
		//		trimmedTargetTorque[k], 
		//		(I32_T)(homeSensorValue[k])
		//		);
		//}
		break;
	}
	
}
void __RtEventCallback( void *UserDataPtr, U32_T EventCode )
{
	RTN_ERR ret;
	USER_DAT *pData = (USER_DAT *)UserDataPtr;
	U16_T state;

	switch( EventCode )
	{
	case EVENT_ECM_STATE_CHANGE:
		ret = NEC_RtGetMasterState( pData->masterId, &state );
		RtPrintf( "State change event! state change to: %d \n", state ); 
		break;
	}
}
void __RtErrorCallback( void *UserDataPtr, I32_T ErrorCode )
{
	RTN_ERR ret;
	USER_DAT *pData = (USER_DAT *)UserDataPtr;

	RtPrintf( "\n\n\n ********************************************************************** \n" );
	RtPrintf( "[x] ERROR!! __RtErrorCallback() ErrorCode = %d \n", ErrorCode );
	RtPrintf( "    Stop master!!\n" );
	RtPrintf( "********************************************************************** \n\n\n" );

	ret = NEC_RtStopMasterCb( pData->masterId );
	if( ret != ECERR_SUCCESS )
	{
		RtPrintf( "__RtErrorCallback:NEC_RtStopMasterCb() error %d \n", ret );
		return;
	}
}

void StartMaster(USER_DAT *pData)
{
	int i;
	ret = NEC_RtInitMaster( pData->masterId );
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_RtInitMaster() error %d \n", ret ); return; }
	
	ret = NEC_CoE402Reset();
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402Reset() error %d \n", ret ); return; }
	
	
	for( i = 0; i<TOTAL_AXIS; ++i )
	{
		ret = NEC_CoE402GetAxisId( pData->masterId, i, &pData->axis[i] ); 
		if( ret != ECERR_SUCCESS ) { RtPrintf( "NEC_CoEGetAxisId() error %d \n", ret ); return; }
	}

	ret = NEC_RtSetParameter( pData->masterId, 0, cycleTime );  // Set master cycle time = 1 ms
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_RtSetParameter() error %d \n", ret ); return; }

	//pData->masterId = masterId;
	ret = NEC_RtGetProcessDataPtr(  pData->masterId , &pData->InputProcessDataPtr, &pData->InPDSizeInByte
		, &pData->OutputProcessDataPtr, &pData->OutPDSizeInByte );
	if( ret != ECERR_SUCCESS ) { RtPrintf( "NEC_RtGetProcessDataPtr() error %d \n", ret ); return; }

	// Register client
	clientParam.version         = NEC_RtRetVer();
	clientParam.userDataPtr     = pData;
	clientParam.cyclicCallback  = __RtCyclicCallback;
	clientParam.eventCallback   = __RtEventCallback;
	clientParam.errorCallback   = __RtErrorCallback;

	ret = NEC_RtRegisterClient( pData->masterId, &clientParam );
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_RtRegisterClient() error %d \n", ret ); return; }

	ret = NEC_RtStartMaster( pData->masterId );
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_RtStartMaster() error %d \n", ret ); return; }
	
	ret = NEC_RtSetMasterStateWait( pData->masterId, ECM_STA_SAFEOP, 15000  ); 
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_RtSetMasterStateWait() error %d \n", ret ); return; }
	
	for( i = 0; i<TOTAL_AXIS; i++ )
	{
		ret = NEC_CoE402UpdatePdoMapping( pData->axis[i] );
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402UpdatePdoMapping() error %d \n", ret ); return; }
		
		ret = NEC_CoE402FaultReset( pData->axis[i], 5000 );
		//if( ret != 0 ) { RtPrintf("NEC_CoE402FaultReset failed! error %d \n", ret  );return; }
	}


	ret = NEC_RtChangeStateToOP( pData->masterId, 5000 );
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_RtChangeStateToOP() error %d \n", ret ); return; }
}
void CloseMaster(USER_DAT *pData)
{
	if( ret != ECERR_SUCCESS )
		RtPrintf( " [Error] Master going to stop..." );

	ret = NEC_RtStopMaster( pData->masterId );
	if( ret != ECERR_SUCCESS ) { RtPrintf( "NEC_RtStopMaster() error %d \n", ret ); }

	ret = NEC_RtUnregisterClient( pData->masterId, &clientParam );
	if( ret != ECERR_SUCCESS ) { RtPrintf( "NEC_RtUnregisterClient() error %d \n", ret ); }

	ret = NEC_CoE402Close();
	if( ret != ECERR_SUCCESS ) { RtPrintf( "NEC_CoE402Close() error %d \n", ret ); }

	ret = NEC_RtCloseMaster( pData->masterId ); 
	if( ret != ECERR_SUCCESS ) { RtPrintf( "NEC_RtCloseMaster() error %d \n", ret ); }

	RtPrintf( "done\n" );
}
RTN_ERR SetMotorParameters(USER_DAT *pData)
{
	int i;
	RTN_ERR ret;

	for(i=0; i<TOTAL_AXIS; i++)
	{
		RtPrintf("%d..", i);
		switch (motor_type[i])
		{  
			case 2342:
				ret = MotorType_2342(pData->axis[i]);
				break;
			case 2619:
				ret = MotorType_2619(pData->axis[i]);
				break;
	
			case 2642:
				ret = MotorType_2642(pData->axis[i]);
				break;

			case 3257:
				ret = MotorType_3257(pData->axis[i]);
				break;
	
			case 3863:
				ret = MotorType_3863(pData->axis[i]);
				break;
	
			case 3890:
				ret = MotorType_3890(pData->axis[i]);
				break;
			default:
				RtPrintf("%d: Wrong motor type", i);
				return 1;
		}
	}
	RtPrintf("\n");
	return ret;
}
RTN_ERR MotorType_2342(CANAxis_T Axis)
{
					
				
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x07);//ControlWord
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x8F);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x03,4,0x64616F6C);//RestoreDefaultParameters_Application
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x06,4,0x64616F6C);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x6402,0x00,2,0x8000);//MotorType
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3910,0x00,1,2);//MOTOR_Microsteps
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3911,0x00,1,0);//MOTOR_Polarity
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3350,0x00,2,2410);//VEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3550,0x00,2,2410);//SVEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3962,0x00,2,64);//Encoder resolution in counts
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x00,2,50000);//PWM_Frequency
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x01,1,1);//PWM_Mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3901,0x00,2,3000);//Motor rated revolutions
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3902,0x00,2,24000);// Motor rated voltage
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3310,0x00,2,80);//PID-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3311,0x00,2,50);//PID-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3312,0x00,2,0);//PID-Controller - differential factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3314,0x00,2,0);//PID-Controller - velocity feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3315,0x00,4,0);//PID-Controller - acceleration feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3510,0x00,2,50);//SVel PI-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3511,0x00,2,50);//SVel PI-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3517,0x00,2,0);//IxR factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

		////////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x3210,0x00,2,213);//PI-Current controller - proportional factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3211,0x00,2,541);//PI-Current controller - integral factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	    ///////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x3221,0x00,4,5000);//Current limit - max. positive
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3223,0x00,4,5000);//Current limit - max. negative
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x00,1,1);//Dynamic current limit I*t - mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x01,4,2100);//Dynamic current limit I*t - peak current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x02,4,1000);//Dynamic current limit I*t - continues current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x03,2,200);//Dynamic current limit I*t - time
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		////////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x6060,0x00,2,4);//Modes of operation  //4: Torque profile mode (tq) //1: Profile position mode (pp) 
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,800);//Motor rated current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		
		/*ret = NEC_CoE402SetParameter(Axis,0x6087,0x00,4,100000);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6088,0x00,2,-1);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
*/
		////////////////////////////////////////////////////////////////////////////////////////////////


		//ret = NEC_CoE402SetParameter(Axis,0x6007,0x00,2,1);//Abort Connection Option Code
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6046,0x02,4,3600);//Maximal velocity
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x01,4,1000);//Velocity acceleration - highest sub-index supported
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x02,2,1);//Velocity acceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x01,4,1000);//Velocity deceleration - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x02,2,1);//Velocity deceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x01,4,1000);//Velocity quickstop - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x02,2,1);//Velocity quickstop - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x01,4,1);//Velocity dimension factor - numerator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x02,4,1);//Velocity dimension factor - denominator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x608C,0x00,1,163);//Velocity dimension index
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x01,4,1);//Feed constant - feed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x02,4,1);//Feed constant - driving shaft revolutions
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x608A,0x00,1,172);//Position dimension index
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6065,0x00,4,10000);//Following error window
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,4300);//Motor rated current
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3340,0x00,4,5000);//Velocity acceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3341,0x00,4,1000);//Velocity acceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3342,0x00,4,3000);//Velocity deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3343,0x00,4,1000);//Velocity deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3344,0x00,4,2000);//Velocity quick stop deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParam20eter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3345,0x00,4,1000);//Velocity quick stop deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x334c,0x00,1,1);//Ramp generator - ramp type
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3521,0x00,2,2147483647);//SVelocity max. limit - positive direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3523,0x00,2,2147483647);//SVelocity max. limit - negative direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3321,0x00,4,5000);//Velocity max. limit - positiv direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3323,0x00,4,5000);//Velocity max. limit - negativ direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

		return 0;
}
RTN_ERR MotorType_2619(CANAxis_T Axis)
{
					
				
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x07);//ControlWord
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x8F);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x03,4,0x64616F6C);//RestoreDefaultParameters_Application
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x06,4,0x64616F6C);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x6402,0x00,2,0x8000);//MotorType
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3910,0x00,1,2);//MOTOR_Microsteps
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3911,0x00,1,0);//MOTOR_Polarity
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3350,0x00,2,2410);//VEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3550,0x00,2,2410);//SVEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3962,0x00,2,64);//Encoder resolution in counts
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x00,2,50000);//PWM_Frequency
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x01,1,1);//PWM_Mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3901,0x00,2,3000);//Motor rated revolutions
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3902,0x00,2,24000);// Motor rated voltage
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3310,0x00,2,80);//PID-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3311,0x00,2,50);//PID-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3312,0x00,2,0);//PID-Controller - differential factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3314,0x00,2,0);//PID-Controller - velocity feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3315,0x00,4,0);//PID-Controller - acceleration feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3510,0x00,2,50);//SVel PI-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3511,0x00,2,50);//SVel PI-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3517,0x00,2,0);//IxR factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

		////////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x3210,0x00,2,1253);//PI-Current controller - proportional factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3211,0x00,2,2781);//PI-Current controller - integral factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	    ///////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x3221,0x00,4,5000);//Current limit - max. positive
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3223,0x00,4,5000);//Current limit - max. negative
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x00,1,1);//Dynamic current limit I*t - mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x01,4,12900);//Dynamic current limit I*t - peak current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x02,4,4300);//Dynamic current limit I*t - continues current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x03,2,200);//Dynamic current limit I*t - time
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		////////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x6060,0x00,2,4);//Modes of operation  //4: Torque profile mode (tq) //1: Profile position mode (pp) 
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,800);//Motor rated current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		
		/*ret = NEC_CoE402SetParameter(Axis,0x6087,0x00,4,100000);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6088,0x00,2,-1);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
*/
		////////////////////////////////////////////////////////////////////////////////////////////////


		//ret = NEC_CoE402SetParameter(Axis,0x6007,0x00,2,1);//Abort Connection Option Code
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6046,0x02,4,3600);//Maximal velocity
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x01,4,1000);//Velocity acceleration - highest sub-index supported
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x02,2,1);//Velocity acceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x01,4,1000);//Velocity deceleration - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x02,2,1);//Velocity deceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x01,4,1000);//Velocity quickstop - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x02,2,1);//Velocity quickstop - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x01,4,1);//Velocity dimension factor - numerator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x02,4,1);//Velocity dimension factor - denominator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x608C,0x00,1,163);//Velocity dimension index
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x01,4,1);//Feed constant - feed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x02,4,1);//Feed constant - driving shaft revolutions
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x608A,0x00,1,172);//Position dimension index
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6065,0x00,4,10000);//Following error window
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,4300);//Motor rated current
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3340,0x00,4,5000);//Velocity acceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3341,0x00,4,1000);//Velocity acceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3342,0x00,4,3000);//Velocity deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3343,0x00,4,1000);//Velocity deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3344,0x00,4,2000);//Velocity quick stop deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParam20eter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3345,0x00,4,1000);//Velocity quick stop deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x334c,0x00,1,1);//Ramp generator - ramp type
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3521,0x00,2,2147483647);//SVelocity max. limit - positive direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3523,0x00,2,2147483647);//SVelocity max. limit - negative direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3321,0x00,4,5000);//Velocity max. limit - positiv direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3323,0x00,4,5000);//Velocity max. limit - negativ direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

		return 0;
}
RTN_ERR MotorType_2642(CANAxis_T Axis)
{	
				
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x07);//ControlWord
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x8F);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x03,4,0x64616F6C);//RestoreDefaultParameters_Application
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x06,4,0x64616F6C);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x6402,0x00,2,0x8000);//MotorType
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3910,0x00,1,2);//MOTOR_Microsteps
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3911,0x00,1,0);//MOTOR_Polarity
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3350,0x00,2,2410);//VEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3550,0x00,2,2410);//SVEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3962,0x00,2,4096);//Encoder resolution in counts
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x00,2,50000);//PWM_Frequency
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x01,1,1);//PWM_Mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3901,0x00,2,3000);//Motor rated revolutions
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3902,0x00,2,24000);// Motor rated voltage
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3310,0x00,2,80);//PID-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3311,0x00,2,50);//PID-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3312,0x00,2,0);//PID-Controller - differential factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3314,0x00,2,0);//PID-Controller - velocity feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3315,0x00,4,0);//PID-Controller - acceleration feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3510,0x00,2,50);//SVel PI-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3511,0x00,2,50);//SVel PI-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3517,0x00,2,0);//IxR factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		
		ret = NEC_CoE402SetParameter(Axis,0x3210,0x00,2, 237);//PI-Current controller - proportional factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3211,0x00,2, 440);//PI-Current controller - integral factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3221,0x00,4,5000);//Current limit - max. positive
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3223,0x00,4,5000);//Current limit - max. negative
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x00,1,1);//Dynamic current limit I*t - mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x01,4,2600);//Dynamic current limit I*t - peak current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x02,4,1200);//Dynamic current limit I*t - continues current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x03,2,200);//Dynamic current limit I*t - time
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		////////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x6060,0x00,2,4);//Modes of operation  //4: Torque profile mode (tq) //1: Profile position mode (pp) 
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,800);//Motor rated current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		/*ret = NEC_CoE402SetParameter(Axis,0x6087,0x00,4,100000);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6088,0x00,2,-1);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
*/
		////////////////////////////////////////////////////////////////////////////////////////////////


		//ret = NEC_CoE402SetParameter(Axis,0x6007,0x00,2,1);//Abort Connection Option Code
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6046,0x02,4,3600);//Maximal velocity
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x01,4,1000);//Velocity acceleration - highest sub-index supported
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x02,2,1);//Velocity acceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x01,4,1000);//Velocity deceleration - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x02,2,1);//Velocity deceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x01,4,1000);//Velocity quickstop - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x02,2,1);//Velocity quickstop - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x01,4,1);//Velocity dimension factor - numerator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x02,4,1);//Velocity dimension factor - denominator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x608C,0x00,1,163);//Velocity dimension index
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x01,4,1);//Feed constant - feed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x02,4,1);//Feed constant - driving shaft revolutions
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x608A,0x00,1,172);//Position dimension index
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6065,0x00,4,10000);//Following error window
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,4300);//Motor rated current
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3340,0x00,4,5000);//Velocity acceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3341,0x00,4,1000);//Velocity acceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3342,0x00,4,3000);//Velocity deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3343,0x00,4,1000);//Velocity deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3344,0x00,4,3000);//Velocity quick stop deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3345,0x00,4,1000);//Velocity quick stop deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x334c,0x00,1,1);//Ramp generator - ramp type
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3521,0x00,2,2147483647);//SVelocity max. limit - positive direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3523,0x00,2,2147483647);//SVelocity max. limit - negative direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3321,0x00,4,6000);//Velocity max. limit - positiv direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3323,0x00,4,6000);//Velocity max. limit - negativ direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

		return 0;
}
RTN_ERR MotorType_3863(CANAxis_T Axis)
{
				
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x07);//ControlWord
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x8F);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x03,4,0x64616F6C);//RestoreDefaultParameters_Application
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x2011,0x06,4,0x64616F6C);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x6402,0x00,2,0x8000);//MotorType
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3910,0x00,1,2);//MOTOR_Microsteps
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3911,0x00,1,0);//MOTOR_Polarity
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3350,0x00,2,2410);//VEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3550,0x00,2,2410);//SVEL_Feedback
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3962,0x00,2,4096);//Encoder resolution in counts
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x00,2,50000);//PWM_Frequency
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3830,0x01,1,1);//PWM_Mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3901,0x00,2,3000);//Motor rated revolutions
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3902,0x00,2,24000);// Motor rated voltage
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3310,0x00,2,80);//PID-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3311,0x00,2,50);//PID-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3312,0x00,2,0);//PID-Controller - differential factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		////為何沒有3313的參數不重要???
		//ret = NEC_CoE402SetParameter(Axis,0x3314,0x00,2,0);//PID-Controller - velocity feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3315,0x00,4,0);//PID-Controller - acceleration feed forward
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3510,0x00,2,50);//SVel PI-Controller - gain
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3511,0x00,2,50);//SVel PI-Controller - integral factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3517,0x00,2,0);//IxR factor
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		
		ret = NEC_CoE402SetParameter(Axis,0x3210,0x00,2,58);//PI-Current controller - proportional factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3211,0x00,2,49);//PI-Current controller - integral factor
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3221,0x00,4,5000);//Current limit - max. positive
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3223,0x00,4,5000);//Current limit - max. negative
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x00,1,1);//Dynamic current limit I*t - mode
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x01,4,12000);//Dynamic current limit I*t - peak current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x02,4,4000);//Dynamic current limit I*t - continues current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3224,0x03,2,200);//Dynamic current limit I*t - time
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		////////////////////////////////////////////////////////////////////////////////////////////////
		ret = NEC_CoE402SetParameter(Axis,0x6060,0x00,2,4);//Modes of operation  //4: Torque profile mode (tq) //1: Profile position mode (pp) 
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,4300);//Motor rated current
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret );return 1; }
		/*ret = NEC_CoE402SetParameter(Axis,0x6087,0x00,4,100000);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6088,0x00,2,-1);
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
*/
		////////////////////////////////////////////////////////////////////////////////////////////////


		//ret = NEC_CoE402SetParameter(Axis,0x6007,0x00,2,1);//Abort Connection Option Code
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		////??????????????????????????????????
		//ret = NEC_CoE402SetParameter(Axis,0x6046,0x02,4,3600);//Maximal velocity
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret );return 1; }

		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x01,4,1000);//Velocity acceleration - highest sub-index supported
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6048,0x02,2,1);//Velocity acceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x01,4,1000);//Velocity deceleration - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6049,0x02,2,1);//Velocity deceleration - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x01,4,1000);//Velocity quickstop - delta speed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604A,0x02,2,1);//Velocity quickstop - delta time
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x01,4,1);//Velocity dimension factor - numerator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x604C,0x02,4,1);//Velocity dimension factor - denominator
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x608C,0x00,1,163);//Velocity dimension index
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x01,4,1);//Feed constant - feed
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x6092,0x02,4,1);//Feed constant - driving shaft revolutions
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x608A,0x00,1,172);//Position dimension index
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		ret = NEC_CoE402SetParameter(Axis,0x6065,0x00,4,10000);//Following error window
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//
		//ret = NEC_CoE402SetParameter(Axis,0x3340,0x00,4,5000);//Velocity acceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3341,0x00,4,1000);//Velocity acceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3342,0x00,4,3000);//Velocity deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3343,0x00,4,1000);//Velocity deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3344,0x00,4,2000);//Velocity quick stop deceleration - delta V
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3345,0x00,4,1000);//Velocity quick stop deceleration - delta T
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x334c,0x00,1,1);//Ramp generator - ramp type
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3521,0x00,2,2147483647);//SVelocity max. limit - positive direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3523,0x00,2,2147483647);//SVelocity max. limit - negative direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
		//ret = NEC_CoE402SetParameter(Axis,0x3321,0x00,4,6000);//Velocity max. limit - positiv direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3323,0x00,4,6000);//Velocity max. limit - negativ direction
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

		return 0;
}
RTN_ERR MotorType_3257(CANAxis_T Axis)
{
	ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x07);//ControlWord - halt on
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x8F);//ControlWord - fault reset
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x2011,0x03,4,0x64616F6C);//RestoreDefaultParameters_Application
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x2011,0x06,4,0x64616F6C);
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x6402,0x00,2,0x8000);//MotorType
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3910,0x00,1,2);//MOTOR_Poles
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3911,0x00,1,0);//MOTOR_Polarity
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3350,0x00,2,2410);//VEL_Feedback
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3550,0x00,2,2410);//SVEL_Feedback
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3962,0x00,2,4096);//Encoder resolution in counts
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3830,0x00,2,50000);//PWM_Frequency							????????????????????
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3830,0x01,1,1);//PWM_Mode
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3901,0x00,2,3000);//Motor rated revolutions
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3902,0x00,2,24000);// Motor rated voltage
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	//ret = NEC_CoE402SetParameter(Axis,0x3310,0x00,2,80);//PID-Controller - gain
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3311,0x00,2,50);//PID-Controller - integral factor
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3312,0x00,2,0);//PID-Controller - differential factor
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//
	//ret = NEC_CoE402SetParameter(Axis,0x3314,0x00,2,0);//PID-Controller - velocity feed forward
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3315,0x00,4,0);//PID-Controller - acceleration feed forward
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

	//ret = NEC_CoE402SetParameter(Axis,0x3510,0x00,2,50);//SVel PI-Controller - gain
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3511,0x00,2,50);//SVel PI-Controller - integral factor
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3517,0x00,2,0);//IxR factor								????????????????????
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

	///////////////////////////////////////////////////////////////////////////////
	ret = NEC_CoE402SetParameter(Axis,0x3210,0x00,2,72);//PI-Current controller - proportional factor
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3211,0x00,2,59);//PI-Current controller - integral factor
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	///////////////////////////////////////////////////////////////////////////////
	
	ret = NEC_CoE402SetParameter(Axis,0x3221,0x00,4,5000);//Current limit - max. positive
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3223,0x00,4,5000);//Current limit - max. negative
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3224,0x00,1,1);//Dynamic current limit I*t - mode
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3224,0x01,4,12900);//Dynamic current limit I*t - peak current???????
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3224,0x02,4,2300);//Dynamic current limit I*t - continues current
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3224,0x03,2,200);//Dynamic current limit I*t - time			????????????????????
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	ret = NEC_CoE402SetParameter(Axis,0x6060,0x00,2,4);//Modes of operation
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,2300);//Motor rated current
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6087,0x00,4,100000);
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6088,0x00,2,-1);
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	////////////////////////////////////////////////////////////////////////////////////////////////

	//ret = NEC_CoE402SetParameter(Axis,0x6007,0x00,2,1);//Abort Connection Option Code
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6046,0x02,4,3600);//Maximal velocity
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6048,0x01,4,1000);//Velocity acceleration - highest sub-index supported
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6048,0x02,2,1);//Velocity acceleration - delta time
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6049,0x01,4,1000);//Velocity deceleration - delta speed
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6049,0x02,2,1);//Velocity deceleration - delta time
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604A,0x01,4,1000);//Velocity quickstop - delta speed
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604A,0x02,2,1);//Velocity quickstop - delta time
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604C,0x01,4,1);//Velocity dimension factor - numerator
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604C,0x02,4,1);//Velocity dimension factor - denominator
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x608C,0x00,1,163);//Velocity dimension index
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6092,0x01,4,1);//Feed constant - feed
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6092,0x02,4,1);//Feed constant - driving shaft revolutions
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x608A,0x00,1,172);//Position dimension index
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6065,0x00,4,10000);//Following error window
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	//ret = NEC_CoE402SetParameter(Axis,0x3340,0x00,4,5000);//Velocity acceleration - delta V
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3341,0x00,4,1000);//Velocity acceleration - delta T
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3342,0x00,4,3000);//Velocity deceleration - delta V
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3343,0x00,4,1000);//Velocity deceleration - delta T
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3344,0x00,4,2000);//Velocity quick stop deceleration - delta V
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3345,0x00,4,1000);//Velocity quick stop deceleration - delta T
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x334c,0x00,1,1);//Ramp generator - ramp type
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//
 //   ret = NEC_CoE402SetParameter(Axis,0x3521,0x00,2,2147483647);//SVelocity max. limit - positive direction
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3523,0x00,2,2147483647);//SVelocity max. limit - negative direction
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//
	//ret = NEC_CoE402SetParameter(Axis,0x3321,0x00,4,5000);//Velocity max. limit - positiv direction
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3323,0x00,4,5000);//Velocity max. limit - negativ direction
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

	return 0;
}
RTN_ERR MotorType_3890(CANAxis_T Axis)
{
	ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x07);//ControlWord - halt on
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6040,0x00,2,0x8F);//ControlWord - fault reset
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x2011,0x03,4,0x64616F6C);//RestoreDefaultParameters_Application
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x2011,0x06,4,0x64616F6C);
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x6402,0x00,2,0x8000);//MotorType a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3910,0x00,1,2);//MOTOR_Poles a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3911,0x00,1,0);//MOTOR_Polarity a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3350,0x00,2,2410);//VEL_Feedback a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3550,0x00,2,2410);//SVEL_Feedback a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3962,0x00,2,4096);//Encoder resolution in counts a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3830,0x00,2,50000);//PWM_Frequency	a						????????????????????
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3830,0x01,1,1);//PWM_Mode a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3901,0x00,2,3600);//Motor rated revolutions a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3902,0x00,2,24000);// Motor rated voltage a
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	//ret = NEC_CoE402SetParameter(Axis,0x3310,0x00,2,80);//PID-Controller - gain b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3311,0x00,2,50);//PID-Controller - integral factor b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3312,0x00,2,0);//PID-Controller - differential factor b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//
	//ret = NEC_CoE402SetParameter(Axis,0x3314,0x00,2,0);//PID-Controller - velocity feed forward b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3315,0x00,4,0);//PID-Controller - acceleration feed forward b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

	//ret = NEC_CoE402SetParameter(Axis,0x3510,0x00,2,50);//SVel PI-Controller - gain b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3511,0x00,2,50);//SVel PI-Controller - integral factor b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3517,0x00,2,0);//IxR factor		b						????????????????????
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

	///////////////////////////////////////////////////////////////////////////////
	ret = NEC_CoE402SetParameter(Axis,0x3210,0x00,2,72);//PI-Current controller - proportional factor c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3211,0x00,2,59);//PI-Current controller - integral factor c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	///////////////////////////////////////////////////////////////////////////////
	
	//ret = NEC_CoE402SetParameter(Axis,0x3221,0x00,4,5000);//Current limit - max. positive
	//TEST
	ret = NEC_CoE402SetParameter(Axis,0x3221,0x00,4,15000);//Current limit - max. positive c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3223,0x00,4,5000);//Current limit - max. negative
	//TEST
	ret = NEC_CoE402SetParameter(Axis,0x3223,0x00,4,15000);//Current limit - max. negative c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

	ret = NEC_CoE402SetParameter(Axis,0x3224,0x00,1,1);//Dynamic current limit I*t - mode c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3224,0x01,4,12900);//Dynamic current limit I*t - peak current
	//TEST
	ret = NEC_CoE402SetParameter(Axis,0x3224,0x01,4,10000);//Dynamic current limit I*t - peak current c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3224,0x02,4,4300);//Dynamic current limit I*t - continues current c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3224,0x03,2,200);//Dynamic current limit I*t - time	c		????????????????????
	//TEST
	ret = NEC_CoE402SetParameter(Axis,0x3224,0x03,2,500);//Dynamic current limit I*t - time		c	????????????????????
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	ret = NEC_CoE402SetParameter(Axis,0x6060,0x00,2,4);//Modes of operation c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6075,0x00,4,4300);//Motor rated current c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6072,0x00,2,1000);//Maximal torque c
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6087,0x00,4,100000);
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6088,0x00,2,-1);
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	////////////////////////////////////////////////////////////////////////////////////////////////

	//ret = NEC_CoE402SetParameter(Axis,0x6007,0x00,2,1);//Abort Connection Option Code
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6046,0x02,4,5400);//Maximal velocity c
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6048,0x01,4,1000);//Velocity acceleration - highest sub-index supported b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6048,0x02,2,1);//Velocity acceleration - delta time b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6049,0x01,4,1000);//Velocity deceleration - delta speed b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6049,0x02,2,1);//Velocity deceleration - delta time b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604A,0x01,4,1000);//Velocity quickstop - delta speed b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604A,0x02,2,1);//Velocity quickstop - delta time b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604C,0x01,4,1);//Velocity dimension factor - numerator b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x604C,0x02,4,1);//Velocity dimension factor - denominator b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x608C,0x00,1,163);//Velocity dimension index b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6092,0x01,4,1);//Feed constant - feed b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x6092,0x02,4,1);//Feed constant - driving shaft revolutions b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x608A,0x00,1,172);//Position dimension index b
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	ret = NEC_CoE402SetParameter(Axis,0x6065,0x00,4,10000);//Following error window b
	if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
	//ret = NEC_CoE402SetParameter(Axis,0x3340,0x00,4,5000);//Velocity acceleration - delta V b 
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3341,0x00,4,100);//Velocity acceleration - delta T b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3342,0x00,4,3000);//Velocity deceleration - delta V b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3343,0x00,4,1000);//Velocity deceleration - delta T b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3344,0x00,4,2000);//Velocity quick stop deceleration - delta V b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3345,0x00,4,1000);//Velocity quick stop deceleration - delta T b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x334c,0x00,1,1);//Ramp generator - ramp type b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//
 //   ret = NEC_CoE402SetParameter(Axis,0x3521,0x00,2,2147483647);//SVelocity max. limit - positive direction b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3523,0x00,2,2147483647);//SVelocity max. limit - negative direction b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//
	//ret = NEC_CoE402SetParameter(Axis,0x3321,0x00,4,5000);//Velocity max. limit - positiv direction b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3323,0x00,4,5000);//Velocity max. limit - negativ direction b
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }

	return 0;
}
void HomingMethod35(USER_DAT *pData)
{
	int i;
	
	//需要先servo on且 6060 home mode才能歸零
	pData->setServoOnFlag = 1;

	for(i=0; i<TOTAL_AXIS; i++)
	{

		ret = NEC_CoE402SetParameter( pData->axis[i], 0x6060, 0x00, 2, 6 );//Modes of operation
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret );}

		ret = NEC_CoE402HomeEx( pData->axis[i], OPT_IMV && OPT_IAC && OPT_IZV && OPT_IOF, 35, 0, 0, 0, 0 );
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402Home() error %d \n", ret );}
		
		pData->firstTimeHoldFlag = 0;
		pData->holdSwitch = 1;

		ret = NEC_CoE402SetParameter( pData->axis[i], 0x6060, 0x00, 2, 4 );//Modes of operation
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret );}
	}
	//pData->setServoOffFlag = 1;

		// TODO
		// check if there is any parameters that could be modified by NEC_CoE402HomeEx() function...
}

void HOMING_UpdateCbTargetTheta(F64_T *CB_targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *homeSensorValue, I32_T *HOMING_cnt)
{
	int i;
	static int homeSensorReachCnt = 0;
	const int DinHallPdoLocalOffset = 10;

	if((*HOMING_cnt) == 0)
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			pData->HOMING_initialTheta[i] = actualTheta[i];
			pData->HOMING_homeSensorReachFlag[i] = 1;
		}


		homeSensorReachCnt = 17;	// 17軸沒作homing, 當作已經reach home sensor了,下面16軸作homing而已

		// trunk
		pData->HOMING_homeSensorReachFlag[4]  = 0;	
		pData->HOMING_homeSensorReachFlag[10] = 0;
		pData->HOMING_homeSensorReachFlag[19] = 0;
		// waist
		pData->HOMING_homeSensorReachFlag[20] = 0;	
		// left leg
		pData->HOMING_homeSensorReachFlag[21] = 0;	
		pData->HOMING_homeSensorReachFlag[22] = 0;
		pData->HOMING_homeSensorReachFlag[23] = 0;
		pData->HOMING_homeSensorReachFlag[24] = 0;
		pData->HOMING_homeSensorReachFlag[25] = 0;
		pData->HOMING_homeSensorReachFlag[26] = 0;
		// right leg
		pData->HOMING_homeSensorReachFlag[27] = 0;	
		pData->HOMING_homeSensorReachFlag[28] = 0;
		pData->HOMING_homeSensorReachFlag[29] = 0;
		pData->HOMING_homeSensorReachFlag[30] = 0;
		pData->HOMING_homeSensorReachFlag[31] = 0;
		pData->HOMING_homeSensorReachFlag[32] = 0;
	}
	else
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			if(pData->HOMING_homeSensorReachFlag[i] == 0)
			{
				NEC_RtGetSlaveProcessDataInput(pData->masterId, i, DinHallPdoLocalOffset, (U8_T*)&homeSensorValue[i], 4);
				if(homeSensorValue[i] == homeSensorReachValue[i])
				{
					pData->HOMING_homeSensorTheta[i] = actualTheta[i];
					RtPrintf("%d = %d\n", i, (I32_T)(pData->HOMING_homeSensorTheta[i]*180/PI) );

					pData->HOMING_homeSensorReachFlag[i] = 1;
					homeSensorReachCnt++;
					if(homeSensorReachCnt == TOTAL_AXIS)
					{
						pData->HOMING_allHomeSensorReachFlag = 1;
						break;
					}

				}
				else
				{
					CB_targetTheta[i] = pData->HOMING_initialTheta[i] + HOMING_maxVelocity[i] * ((*HOMING_cnt) * pData->cycleTime * MILISECOND_TO_SECOND);
				}
			}
		}
	}
	(*HOMING_cnt)++;

}
void PP_UpdateCbTargetTheta(F64_T *CB_targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *PP_cnt)
{
	int i;

	if((*PP_cnt) == 0)
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			pData->PP_initialTheta[i] = actualTheta[i];
		}
	}
	else
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			CB_targetTheta[i] = pData->PP_initialTheta[i] + (pData->PP_targetTheta[i] - pData->PP_initialTheta[i]) * pData->PP_splineVec[(*PP_cnt)];
		}
	}
	(*PP_cnt)++;

}
void CSP_UpdateCbTargetTheta(F64_T *CB_targetTheta, USER_DAT *pData, F64_T *actualTheta, I32_T *CSP_cnt)
{
	int i;

	for(i=0; i<TOTAL_AXIS; i++)
	{
		if(i==3 || i== 5 || i==8 || i==11 || i==19 || i==20 || i>=21)
		{
			CB_targetTheta[i] = pData->WalkingTrajectories[floor((*CSP_cnt) * pData->walkingSpeed)][i];
			pData->ActualWalkingTrajectories[(*CSP_cnt)][i] = actualTheta[i];
		}
	}
	(*CSP_cnt)++;
}

I16_T TargetTorqueTrimming(F64_T tempTorque)
{
	if(tempTorque>=1000.0) return (I16_T)1000;
	else if(tempTorque<=-1000.0) return (I16_T)-1000;
	else return (I16_T)tempTorque;
}