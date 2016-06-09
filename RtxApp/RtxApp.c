#include "RtxApp.h"

USER_DAT	*pData;


void 
_cdecl
main(
       int  argc,
       char **argv,
       char **envp
    )
{
	HANDLE		sHhandle;
	HANDLE		oBhandle[EVN_NUM];
	int			i;

	// create memory
	sHhandle = RtCreateSharedMemory ((DWORD) PAGE_READWRITE , (DWORD) 0 , (DWORD) sizeof(USER_DAT) , SHM_NAME , &location);
	if( sHhandle == NULL )
	{
		RtPrintf("Create RtCreateSharedMemory fail\n");
		ExitProcess(0);
	}
	pData = (USER_DAT *) (location);
	
	//create event
	for(i=0; i<EVN_NUM; i++)
	{
		oBhandle[i] = RtCreateEvent( NULL, 0, FALSE, EVN_NAME[i] );
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


	RtPrintf("===========================\n");
	RtPrintf("RTX_READY\n");
	RtPrintf("===========================\n\n");

	while(1)
	{
		ret = RtWaitForSingleObject( oBhandle[START_MASTER_AND_SLAVES], 5 );	//oBhandle:要等待的觸發事; >0: milisecond, -1:waiting forever
		if( ret == 0 )
		{
			RtPrintf("START_MASTER_AND_SLAVES...\n");
			StartMaster(pData);
			RtPrintf("done\n\n");
		}

		ret = RtWaitForSingleObject( oBhandle[SET_MOTOR_PARAMETERS], 5 );
		if( ret == 0 )
		{
			RtPrintf( "SET_MOTOR_PARAMETERS...\n" );
			SetMotorParameters(pData);
			RtPrintf("done\n\n");
		}

		ret = RtWaitForSingleObject( oBhandle[SET_CURR_POS_HOME], 5 );
		if( ret == 0 )
		{
			RtPrintf( "SET_CURR_POS_HOME...\n" );
			HomingMethod35(pData);
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
	static I32_T	PP_cnt = 0;
	static I32_T	HOMING_cnt = 0;
	static I32_T	CSP_cnt = 0;
	static I32_T	FTS_cnt = 0;
	static I32_T	OPG_cnt = 0;

	// state
	I32_T			actualEncoderPos[TOTAL_AXIS];
	F64_T			CB_actualTheta[TOTAL_AXIS];
	static F64_T	CB_targetTheta[TOTAL_AXIS];
	static F64_T	VirtualHomeTheta[TOTAL_AXIS];


	// homing related
	I32_T			home_sensor_value[TOTAL_AXIS];




	NEC_RtGetMasterState( pData->masterId, &state );
	switch( state )
	{
	case ECM_STA_OPERATION:
		NEC_CoE402CyclicProcess();
		
		// variabls needed to be updated every cycle
		for(k=0; k<TOTAL_AXIS; k++) 
		{
			NEC_CoE402GetActualPosition( pData->axis[k], &actualEncoderPos[k]);
			CB_actualTheta[k] = ((F64_T)actualEncoderPos[k]) / axis_theta_to_motor_resolution[k] - VirtualHomeTheta[k];	
		}

		// flags
		if(pData->Flag_ServoOn == 1)
		{
			for(k=0; k<TOTAL_AXIS; k++) 
			{
				NEC_CoE402SetState( pData->axis[k], 3);
			}
			pData->Flag_ServoOn = 0;
		}
		if(pData->Flag_ServoOff == 1)
		{
			for(k=0; k<TOTAL_AXIS; k++) 
			{
				NEC_CoE402SetState( pData->axis[k], 0);
			}
			pData->Flag_ServoOff = 0;
		}
		if(pData->Flag_UpdateActualTheta == 1)
		{
			for(k=0; k<TOTAL_AXIS; k++) 
			{
				pData->actualTheta[k] = CB_actualTheta[k];
			}
			pData->Flag_UpdateActualTheta = 0;
		}
		if(pData->Flag_SetCurrPosHome == 1)
		{
			for(k=0; k<TOTAL_AXIS; k++) 
			{
				VirtualHomeTheta[k] = ((F64_T)actualEncoderPos[k]) / axis_theta_to_motor_resolution[k];
				CB_actualTheta[k] = 0;//CB_actualTheta[k] - VirtualHomeTheta[k];
				CB_targetTheta[k] = 0;
			}
			pData->Flag_SetCurrPosHome = 0;
		}
		if(pData->Flag_ResetCnt == 1)
		{
			HOMING_cnt = 0;
			PP_cnt = 0;
			CSP_cnt = 0;
			FTS_cnt = 0;
			OPG_cnt = 0;
			pData->Flag_ResetCnt = 0;
		}


		// Motor State
		switch(pData->MotorState)
		{
		case MotorState_NoTq:
			for(k=0; k<TOTAL_AXIS; k++)
			{
				NEC_CoE402SetTargetTorque( pData->axis[k], 0 );
			}
			break;

		case MotorState_Hold:
			if(pData->Flag_HoldPosSaved == 0)
			{	
				SaveHoldPos(CB_targetTheta, CB_actualTheta);
				pData->Flag_HoldPosSaved = 1;
			}

			MotorPosPidControl(CB_targetTheta, CB_actualTheta, pData);
			break;

		case MotorState_Homing:
			if(pData->Flag_AllHomeSensorReached == 0)
			{
				HOMING_UpdateCbTargetTheta(CB_targetTheta, CB_actualTheta, home_sensor_value, &HOMING_cnt);
			}
			MotorPosPidControl(CB_targetTheta, CB_actualTheta, pData);
			break;

		case MotorState_PP:
			if((pData->Flag_PpReachTarget == 1) && (pData->PP_Queue_Front != pData->PP_Queue_Rear)) // reach target and PP_Queue is not empty
			{
				pData->Flag_ResetCnt = 1;
				pData->Flag_PpReachTarget = 0;
				pData->PP_Queue_Front = (pData->PP_Queue_Front + 1) % PP_QUEUE_SIZE;	// pop PP_Queue
			}
			else if(pData->Flag_PpReachTarget == 0)
			{
				PP_UpdateCbTargetTheta(CB_targetTheta, CB_actualTheta, &PP_cnt);
			}

			MotorPosPidControl(CB_targetTheta, CB_actualTheta, pData);
			break;

		case MotorState_CSP:
			if(pData->Flag_CspFinished == 0)
			{
				CSP_UpdateCbTargetTheta(CB_targetTheta, CB_actualTheta, &CSP_cnt);
			}
			MotorPosPidControl(CB_targetTheta, CB_actualTheta, pData);
			break;

		case MotorState_FtsTest:
			Fts_UpdateCbTargetTheta(CB_targetTheta, CB_actualTheta, &FTS_cnt);
			MotorPosPidControl(CB_targetTheta, CB_actualTheta, pData);
			break;

		case MotorState_OPG:
			OPG_UpdateTargetPose(&OPG_cnt);
			//UpdateIK_FK();
			//GravityCompensation();
			//MotorPosPidControl(CB_targetTheta, CB_actualTheta, pData);
			break;
		}

		cnt++;

		
		//if( ( cnt % 30 ) == 0 )
		//{
		//	RtPrintf(" %d, %d, %d, %d\n", pData->supportState, (I32_T)pData->fz[0], (I32_T)pData->fz[1], pData->DoubleSupport_cnt);
		//	//RtPrintf("    %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", (I32_T)pData->mx[0], (I32_T)pData->my[0], (I32_T)pData->mz[0], (I32_T)pData->fx[0], (I32_T)pData->fy[0], (I32_T)pData->fz[0], (I32_T)pData->mx[1], (I32_T)pData->my[1], (I32_T)pData->mz[1], (I32_T)pData->fx[1], (I32_T)pData->fy[1], (I32_T)pData->fz[1], (I32_T)pData->FzThreshold);
		//
		////	k=32;
		////	RtPrintf("  CB_target = %d, VirtualHomeTheta = %d, CB_actual = %d\n", (I32_T)(CB_targetTheta[k]*180.0/PI), (I32_T)(VirtualHomeTheta[k]*180.0/PI), (I32_T)(CB_actualTheta[k]*180.0/PI));
		//////	//NEC_RtGetProcessDataInput(pData->masterId, 679, 4, (U8_T*)&home_sensor_value[k]);
		//////	NEC_RtGetSlaveProcessDataInput(pData->masterId, k, 10, (U8_T*)&home_sensor_value[k], 4);
		//////	RtPrintf("       %d, targetTheta %d, actualTheta %d, actualTorque %d, test: %d\n", 
		//////		k, 
		//////		(I32_T)(targetTheta[k]*180.0/PI),
		//////		(I32_T)(actualTheta[k]*180.0/PI), 
		//////		trimmedTargetTorque[k], 
		//////		(I32_T)(home_sensor_value[k])
		//////		);
		////	RtPrintf("    %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", (I32_T)pData->mx[0], (I32_T)pData->my[0], (I32_T)pData->mz[0], (I32_T)pData->fx[0], (I32_T)pData->fy[0], (I32_T)pData->fz[0], (I32_T)pData->mx[1], (I32_T)pData->my[1], (I32_T)pData->mz[1], (I32_T)pData->fx[1], (I32_T)pData->fy[1], (I32_T)pData->fz[1], (I32_T)pData->FzThreshold);
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
	pData->Flag_SetMotorParameterDone = 1;
	
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
	
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
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
	
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
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
	
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
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
	
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
		//ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
		//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
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
	
	//ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
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
	
	//ret = NEC_CoE402SetParameter(Axis,0x3720,0x00,4,-2147483648);//Position limit - minimum c
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	//ret = NEC_CoE402SetParameter(Axis,0x3720,0x01,4,2147483647);//Position limit - maximum c
	//if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret ); return 1; }
	
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

	pData->Flag_StartMasterDone = 1;
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
void HomingMethod35(USER_DAT *pData)
{
	int i;
	
	//需要先servo on且 6060 home mode才能歸零
	pData->Flag_ServoOn = 1;

	for(i=0; i<TOTAL_AXIS; i++)
	{

		ret = NEC_CoE402SetParameter( pData->axis[i], 0x6060, 0x00, 2, 6 );//Modes of operation
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret );}

		ret = NEC_CoE402HomeEx( pData->axis[i], OPT_IMV && OPT_IAC && OPT_IZV && OPT_IOF, 35, 0, 0, 0, 0 );
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402Home() error %d \n", ret );}
		
		pData->Flag_HoldPosSaved = 0;
		pData->MotorState = MotorState_Hold;

		ret = NEC_CoE402SetParameter( pData->axis[i], 0x6060, 0x00, 2, 4 );//Modes of operation
		if( ret != ECERR_SUCCESS ){ RtPrintf( "NEC_CoE402SetParameter() error %d \n", ret );}
	}
	
	//pData->Flag_SetCurrPosHomeDone = 1;
	
}
void SaveHoldPos(F64_T *CB_targetTheta, F64_T *CB_actualTheta)
{
	int k;
	for(k=0; k<TOTAL_AXIS; k++) 
	{	
		CB_targetTheta[k] = CB_actualTheta[k];
	}
}
void HOMING_UpdateCbTargetTheta(F64_T *CB_targetTheta, F64_T *CB_actualTheta, I32_T *home_sensor_value, I32_T *HOMING_cnt)
{
	int i;
	static int homeSensorReachCnt = 0;
	const int DinHallPdoLocalOffset = 10;
	static F64_T HOMING_initialTheta[TOTAL_AXIS];
	
	if((*HOMING_cnt) == 0)
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			HOMING_initialTheta[i] = CB_actualTheta[i];
			pData->Flag_HomeSensorReached[i] = TRUE;
		}

		homeSensorReachCnt = 17;	// 17軸沒作homing, 當作已經reach home sensor, 只有下面16軸作homing而已

		// trunk
		pData->Flag_HomeSensorReached[4]  = FALSE;	
		pData->Flag_HomeSensorReached[10] = FALSE;
		pData->Flag_HomeSensorReached[19] = FALSE;
		// waist
		pData->Flag_HomeSensorReached[20] = FALSE;	
		// left leg
		pData->Flag_HomeSensorReached[21] = FALSE;	
		pData->Flag_HomeSensorReached[22] = FALSE;
		pData->Flag_HomeSensorReached[23] = FALSE;
		pData->Flag_HomeSensorReached[24] = FALSE;
		pData->Flag_HomeSensorReached[25] = FALSE;
		pData->Flag_HomeSensorReached[26] = FALSE;
		// right leg
		pData->Flag_HomeSensorReached[27] = FALSE;
		pData->Flag_HomeSensorReached[28] = FALSE;
		pData->Flag_HomeSensorReached[29] = FALSE;
		pData->Flag_HomeSensorReached[30] = FALSE;
		pData->Flag_HomeSensorReached[31] = FALSE;
		pData->Flag_HomeSensorReached[32] = FALSE;
	}
	else
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			if(pData->Flag_HomeSensorReached[i] == FALSE)
			{
				NEC_RtGetSlaveProcessDataInput(pData->masterId, i, DinHallPdoLocalOffset, (U8_T*)&home_sensor_value[i], 4);
				if(home_sensor_value[i] == homeSensorReachValue[i])
				{
					pData->HOMING_homeSensorTheta[i] = CB_actualTheta[i];
					RtPrintf("%d = %d, ", i, (I32_T)(pData->HOMING_homeSensorTheta[i]*180/PI) );

					pData->Flag_HomeSensorReached[i] = 1;
					homeSensorReachCnt++;
					if(homeSensorReachCnt == TOTAL_AXIS)
					{
						pData->Flag_AllHomeSensorReached = 1;
						break;
					}
				}
				else
				{
					CB_targetTheta[i] = HOMING_initialTheta[i] + HOMING_maxVelocity[i] * ((*HOMING_cnt) * pData->cycleTime * MICROSECOND_TO_SECOND);
				}
			}
		}
	}
	(*HOMING_cnt)++;

}
void PP_UpdateCbTargetTheta(F64_T *CB_targetTheta, F64_T *CB_actualTheta, I32_T *PP_cnt)
{
	int i;
	static F64_T PP_initialTheta[TOTAL_AXIS];

	if((*PP_cnt) == 0)
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			PP_initialTheta[i] = CB_actualTheta[i];
		}
	}

	if((*PP_cnt) < pData->PP_Queue_TimePeriod[pData->PP_Queue_Front] / (pData->cycleTime * MICROSECOND_TO_SECOND) - 1)
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			CB_targetTheta[i] = PP_initialTheta[i] + 
								(pData->PP_Queue_TargetTheta[pData->PP_Queue_Front][i] - PP_initialTheta[i]) 
									* pData->CubicPolyVec[(int)floor((*PP_cnt) * MAX_MOTION_TIME_FRAME * MICROSECOND_TO_SECOND * pData->cycleTime / pData->PP_Queue_TimePeriod[pData->PP_Queue_Front])];
		}

		(*PP_cnt)++;
	}
	else
	{
		pData->Flag_PpReachTarget = 1;
	}
}
void CSP_UpdateCbTargetTheta(F64_T *CB_targetTheta, F64_T *CB_actualTheta, I32_T *CSP_cnt)
{
	
	/*
	int i, k;
	int prepare_cnt = 150; // 0.3 second
	//int adaptive_cnt = 300;
	int PC_prepare_cnt = 500;

	static BOOL_T	Flag_touchGround[2];	// 1:left 2:right
	static BOOL_T	Flag_adapteToGround[2];	// 1:left 2:right
	static double LAR_adaptive_offset;
	static double LAP_adaptive_offset;
	static double RAR_adaptive_offset;
	static double RAP_adaptive_offset;
	static int FTS_cnt;
	//static int DoubleSupport_cnt;




	if((*CSP_cnt) == 0)
	{
		for(i=0; i<2; i++)
		{
			Flag_touchGround[i] = 1;
			Flag_adapteToGround[i] = 1;
		}
		LAR_adaptive_offset = 0;
		LAP_adaptive_offset = 0;
		RAR_adaptive_offset = 0;
		RAP_adaptive_offset = 0;
		FTS_cnt = 0;
		
		// check if CB_targetThet != walking initial pose
	}

	if((*CSP_cnt) * pData->walkingSpeed < pData->walkingTimeframe)
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			if(i==25)
				CB_targetTheta[i] = pData->WalkingTrajectories[(int)floor((*CSP_cnt) * pData->walkingSpeed)][i] + LAP_adaptive_offset;
			else if(i==26)
				CB_targetTheta[i] = pData->WalkingTrajectories[(int)floor((*CSP_cnt) * pData->walkingSpeed)][i] + LAR_adaptive_offset;
			else if(i==31)
				CB_targetTheta[i] = pData->WalkingTrajectories[(int)floor((*CSP_cnt) * pData->walkingSpeed)][i] + RAP_adaptive_offset;
			else if(i==32)
				CB_targetTheta[i] = pData->WalkingTrajectories[(int)floor((*CSP_cnt) * pData->walkingSpeed)][i] + RAR_adaptive_offset;
			else if(i==3 || i== 5 || i==8 || i==11 || i==19 || i==20 || i>=21)
			{
				CB_targetTheta[i] = pData->WalkingTrajectories[(int)floor((*CSP_cnt) * pData->walkingSpeed)][i];
				//pData->ActualWalkingTrajectories[(*CSP_cnt)][i] = CB_actualTheta[i];
			}
		}

/*
		// prepare_time before swing leg landing
		if(((int)floor(((*CSP_cnt) - PC_prepare_cnt) * pData->walkingSpeed*2) + prepare_cnt) % (I32_T)(pData->S_Step / ((pData->cycleTime) * MICROSECOND_TO_SECOND)) == 0)
		{
			if((*CSP_cnt) - PC_prepare_cnt < 0)
			{}
			else if(((int)floor(((*CSP_cnt) - PC_prepare_cnt) * pData->walkingSpeed*2) + prepare_cnt) / (I32_T)(pData->S_Step / ((pData->cycleTime) * MICROSECOND_TO_SECOND)) % 2 == 0)
			{
				// before left foot landing
				RtPrintf("0  %d %d\n", (I32_T)floor(((*CSP_cnt) - PC_prepare_cnt) * pData->walkingSpeed), (I32_T)(pData->S_Step / ((pData->cycleTime) * MICROSECOND_TO_SECOND)));
				
				Flag_touchGround[0] = 0;
				Flag_adapteToGround[0] = 0;
				FTS_cnt++;
			}
			else if(((int)floor(((*CSP_cnt) - PC_prepare_cnt) * pData->walkingSpeed*2) + prepare_cnt) / (I32_T)(pData->S_Step / ((pData->cycleTime) * MICROSECOND_TO_SECOND)) % 2 == 1)
			{
				// before left foot landing
				RtPrintf("1  %d %d\n", (I32_T)floor(((*CSP_cnt) - PC_prepare_cnt) * pData->walkingSpeed), (I32_T)(pData->S_Step / ((pData->cycleTime) * MICROSECOND_TO_SECOND)));				
			
				Flag_touchGround[1] = 0;
				Flag_adapteToGround[1] = 0;
				FTS_cnt++;
			}
		}
		else if(((int)floor(((*CSP_cnt) - PC_prepare_cnt) * pData->walkingSpeed*2) - pData->adaptive_cnt) % (I32_T)(pData->S_Step / ((pData->cycleTime) * MICROSECOND_TO_SECOND)) == 0)
		{
			for(i=0; i<2; i++)
			{
				Flag_touchGround[i] = 1;
				Flag_adapteToGround[i] = 1;
				FTS_cnt = 0;
			}
		}

		k=0;	
		if(FTS_cnt != 0 && Flag_touchGround[k] == 0 && abs(pData->fz[k]) < pData->FzThreshold)
		{
			//RtPrintf("XX 1\n");
		}
		else if(Flag_touchGround[k] == 0 && abs(pData->fz[k]) >= pData->FzThreshold)
		{
			Flag_touchGround[k] = 1;
			//RtPrintf("XX 2\n");
		}
		else if(Flag_touchGround[k] == 1 && Flag_adapteToGround[k] == 0 && (abs(pData->mx[k]) >= pData->MxyThreshold || abs(pData->my[k]) >= pData->MxyThreshold))
		{
			LAR_adaptive_offset += pData->Fts_LRK * (F64_T)pData->mx[k];
			LAP_adaptive_offset += pData->Fts_LPK * (F64_T)pData->my[k];
			FTS_cnt++;
		}
		else if(Flag_touchGround[k] == 1 && Flag_adapteToGround[k] == 0 && abs(pData->fz[k]) > pData->FzThreshold && (abs(pData->mx[k]) < pData->MxyThreshold && abs(pData->my[k]) < pData->MxyThreshold))
		{
			Flag_adapteToGround[k] = 1;
			FTS_cnt = 0;
			//RtPrintf("XX 4\n");
		}
		
		k=1;
		if(FTS_cnt != 0 && Flag_touchGround[k] == 0 && abs(pData->fz[k]) < pData->FzThreshold)
		{
			//RtPrintf("XX 1\n");
		}
		else if(Flag_touchGround[k] == 0 && abs(pData->fz[k]) >= pData->FzThreshold)
		{
			Flag_touchGround[k] = 1;
			//RtPrintf("XX 2\n");
		}
		else if(Flag_touchGround[k] == 1 && Flag_adapteToGround[k] == 0 && (abs(pData->mx[k]) >= pData->MxyThreshold || abs(pData->my[k]) >= pData->MxyThreshold))
		{
			RAR_adaptive_offset += pData->Fts_RRK * (F64_T)pData->mx[k];
			RAP_adaptive_offset += pData->Fts_RPK * (F64_T)pData->my[k];
			FTS_cnt++;
		}
		else if(Flag_touchGround[k] == 1 && Flag_adapteToGround[k] == 0 && abs(pData->fz[k]) > pData->FzThreshold && (abs(pData->mx[k]) < pData->MxyThreshold && abs(pData->my[k]) < pData->MxyThreshold))
		{
			Flag_adapteToGround[k] = 1;
			FTS_cnt = 0;
			//RtPrintf("XX 4\n");
		}
*/
		//	recognizing support leg
		//
		//if(abs(pData->fz[0]) > pData->FzThreshold && abs(pData->fz[1]) < pData->FzThreshold && pData->DoubleSupport_cnt > 125)
		//{
		//	pData->DoubleSupport_cnt = 0;
		//	pData->supportState = 0;
		//}
		//else if(abs(pData->fz[0]) < pData->FzThreshold && abs(pData->fz[1]) > pData->FzThreshold && pData->DoubleSupport_cnt > 125)
		//{
		//	pData->DoubleSupport_cnt = 0;
		//	pData->supportState = 1;
		//}
		//else if(abs(pData->fz[0]) > pData->FzThreshold && abs(pData->fz[1]) > pData->FzThreshold)
		//{
		//	pData->supportState = 2;
		//	(pData->DoubleSupport_cnt)++;
		//}
		//else if(abs(pData->fz[0]) < pData->FzThreshold && abs(pData->fz[1]) < pData->FzThreshold)
		//{
		//	pData->DoubleSupport_cnt = 0;
		//	pData->supportState = 3;
		//}
/*
		LAR_adaptive_offset += pData->Fts_LRK * (F64_T)pData->mx[0];
		LAP_adaptive_offset += pData->Fts_LPK * (F64_T)pData->my[0];
		RAP_adaptive_offset += pData->Fts_RRK * (F64_T)pData->mx[1];
		RAP_adaptive_offset += pData->Fts_RPK * (F64_T)pData->my[1];

		(*CSP_cnt)++;
	}
	else
	{
		for(i=0; i<2; i++)
		{
			Flag_touchGround[i] = 1;
			Flag_adapteToGround[i] = 1;
			FTS_cnt = 0;
		}
		pData->Flag_CspFinished = 1;
	}
*/
	//  Original
	int i;
	if((*CSP_cnt) == 0)
	{
		// check if CB_targetThet != walking initial pose
	}

	if((*CSP_cnt) * pData->walkingSpeed < pData->walkingTimeframe)
	{
		for(i=0; i<TOTAL_AXIS; i++)
		{
			if(i==3 || i== 5 || i==8 || i==11 || i==19 || i==20 || i>=21)
			{
				CB_targetTheta[i] = pData->WalkingTrajectories[(int)floor((*CSP_cnt) * pData->walkingSpeed)][i];
				pData->ActualWalkingTrajectories[(*CSP_cnt)][i] = CB_actualTheta[i];
			}
		}
		(*CSP_cnt)++;
	}
	else
	{
		pData->Flag_CspFinished = 1;
	}
}
void Fts_UpdateCbTargetTheta(F64_T *CB_targetTheta, F64_T *CB_actualTheta, I32_T *FTS_cnt)
{
	int i;
	int k=1;
	static BOOL_T	Flag_touchGround[2];	// 1:left 2:right
	static BOOL_T	Flag_adapteToGround[2];	// 1:left 2:right

	int LAR = 26, LAP = 25, RAR = 32, RAP = 31;		// left/right ankle roll/pitch axis number

	
	if((*FTS_cnt) == 0)
	{
		for(i=0; i<2; i++)
		{
			Flag_touchGround[i] = 0;
			Flag_adapteToGround[i] = 0;
		}
		(*FTS_cnt)++;
		RtPrintf("XX 0\n");
	}
	
	if((*FTS_cnt) != 0 && Flag_touchGround[k] == 0 && abs(pData->fz[k]) < pData->FzThreshold)
	{
		RtPrintf("XX 1\n");
	}
	else if(Flag_touchGround[k] == 0 && abs(pData->fz[k]) >= pData->FzThreshold)
	{
		Flag_touchGround[k] = 1;
		RtPrintf("XX 2\n");
	}
	else if(Flag_touchGround[k] == 1 && Flag_adapteToGround[k] == 0 && (abs(pData->mx[k]) >= pData->MxyThreshold || abs(pData->my[k]) >= pData->MxyThreshold))
	{
		CB_targetTheta[RAR] = CB_actualTheta[RAR] + pData->Fts_RRK * (F64_T)pData->mx[k];
		CB_targetTheta[RAP] = CB_actualTheta[RAP] + pData->Fts_RPK * (F64_T)pData->my[k];

		(*FTS_cnt)++;
	}
	else if(Flag_touchGround[k] == 1 && Flag_adapteToGround[k] == 0 && abs(pData->fz[k]) > pData->FzThreshold && (abs(pData->mx[k]) < pData->MxyThreshold && abs(pData->my[k]) < pData->MxyThreshold))
	{
		Flag_adapteToGround[k] = 1;
		RtPrintf("XX 4\n");
	}
	
	CB_targetTheta[LAR] = CB_actualTheta[LAR] + pData->Fts_LRK * (F64_T)pData->mx[0];
	CB_targetTheta[LAP] = CB_actualTheta[LAP] + pData->Fts_LPK * (F64_T)pData->my[0];
}
void OPG_UpdateTargetPose(I32_T *OPG_cnt)
{
	double sampling_time	= 0.002;
	double step_time		= 0.5;
	double curr_time		= (*OPG_cnt) * sampling_time;

	double cog_height		= 70;
	double G				= 980.665;
	double omega			= pow(G / cog_height, 0.5);
	double cp_offset		= 2;
	double foot_distance	= 25;
	double step_length		= 10;
	double swing_leg_height = 3;
	double b				= exp(omega * step_time);

	int sup_leg				= 1;


}

I16_T TargetTorqueTrimming(F64_T tempTorque)
{
	if(tempTorque>=1000.0) return (I16_T)1000;
	else if(tempTorque<=-1000.0) return (I16_T)-1000;
	else return (I16_T)tempTorque;
}
void MotorPosPidControl(F64_T *CB_targetTheta, F64_T *CB_actualTheta, USER_DAT *pData)
{
	int k;
	F64_T			errorTheta[TOTAL_AXIS];
	F64_T			errorThetaDot[TOTAL_AXIS];
	static F64_T	preErrorTheta[TOTAL_AXIS];
	static F64_T	errorThetaSum[TOTAL_AXIS];
	F64_T			targetTorque[TOTAL_AXIS];
	I16_T			trimmedTargetTorque[TOTAL_AXIS];

	if(pData->Flag_ResetCbErrorTheta == 1)
	{
		for(k=0; k<TOTAL_AXIS; k++)
		{
			preErrorTheta[k] = 0;
			errorThetaSum[k] = 0;
		}
		pData->Flag_ResetCbErrorTheta = 0;
	}

	for(k=0; k<TOTAL_AXIS; k++)
	{
		errorTheta[k] = CB_targetTheta[k] - CB_actualTheta[k];
		if(errorThetaSum[k] < maxErrorThetaSum)
			errorThetaSum[k] = errorThetaSum[k] + errorTheta[k];
		errorThetaDot[k] = (errorTheta[k] - preErrorTheta[k]) / (((F64_T)pData->cycleTime) * MICROSECOND_TO_SECOND);
		preErrorTheta[k] = errorTheta[k];
		targetTorque[k] = (pData->Kp[k] * errorTheta[k] + pData->Ki[k] * errorThetaSum[k] + pData->Kd[k]  * errorThetaDot[k]) * motor_direction[k];
					
		if(pData->motorTorqueSwitch[k] == 0) targetTorque[k] = 0;
		trimmedTargetTorque[k] = TargetTorqueTrimming( targetTorque[k] );

		// !!
		NEC_CoE402SetTargetTorque( pData->axis[k], trimmedTargetTorque[k] );
		// !!
	}
}