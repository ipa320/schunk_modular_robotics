/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: Generic_CANOpen
 * ROS stack name: cob_drivers
 * ROS package name: cob_generic_can
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: 
 * Supervised by:
 *
 * Date of creation: Jan 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

// includes
#include <CANOpenMaster.h>

/*---------------------------------------------------------------*/
// starts SYNC Thread
void InitMaster()
{		
	// access CAN hardware
	// TODO get rid of *.ini
	can_itf = new CANPeakSysUSB("/home/tim/git/care-o-bot/schunk_modular_robotics/CANOpen_driver/CanCtrl.ini"); 
 
	// Init Buffer 
	PDO_buffer = new CANOpen_buffer(); 
}

/*---------------------------------------------------------------*/
// starts SYNC Thread
void StartMasterThread(double SYNC_Freq_HZ)
{			
	// init threads 
	pthread_t thread_Master;
	int thread_Master_ret; 
	
	TD_Master.Cycle_Freq_HZ = SYNC_Freq_HZ;

	thread_Master_ret = pthread_create( &thread_Master, NULL, MasterThreadFunc,(void*) &TD_Master); 	
}

/*---------------------------------------------------------------*/
// activate the PDO 
void AcitvatePDO(int CAN_ID, int number, double SYNC_Freq_HZ)
{	
	if (PDO_buffer->IsActive(number))
	{	PDO_buffer->RegMsg(CAN_ID, number, SYNC_Freq_HZ);	}
}

/*---------------------------------------------------------------*/
// set new value to PDO
void AssignValue2PDO(int number,char* Msg)
{	
	// Msg array has fixed lenght of 8 		
	PDO_buffer->SetMsg(number, Msg);
}

/*---------------------------------------------------------------*/
// get Msg buffer
int GetRecievedMsgs(CanMsg* Msgs)
{	
	int Msg_Counter = 0; 

	for(int i=0; i<MAX_InBuffer_Lenght; i++) 
	{	
		if(!InBuffer.empty())
		{	Msgs[i] = InBuffer.front();
			InBuffer.pop();
			Msg_Counter++; 
		} 
	}
	return Msg_Counter;	
}

/*=============================================*/

/*---------------------------------------------------------------*/
// starts SYNC Thread
void *MasterThreadFunc(void* TD_Master)
{			
	// cast Data struct
	Thread_data* m_TD = (Thread_data*) TD_Master; 

	// init threads 
	pthread_t thread_WorkBuffer;
	int thread_WorkBuffer_ret; 
	Mutex* mtx; 	

	mtx = new Mutex(); 

	for(;;)
	{
		mtx->lock(); 
		// TODO scaduling of the worker 
		thread_WorkBuffer_ret = pthread_create( &thread_WorkBuffer, NULL,EvaluateBuffers,(void*) TD_Master); 	
	
		mtx->unlock();

		usleep(1000000/(double)m_TD->Cycle_Freq_HZ);
	}
	pthread_exit(0);
}

/*---------------------------------------------------------------*/
// Loop for buffer workout
void *EvaluateBuffers(void* TD)
{	
	int Msg_Counter = 0; 
	CanMsg cMsg; 

	Thread_data* m_TD = (Thread_data*) TD; 
	
	
	
	// 1. send all PDOs on the bus
	for (int i=0;i<MAX_PDOS;i++)
	{
		if(PDO_buffer->IsActive(i))
		{		
			// send PDO value 
			//WritePDO(PDO_buffer->data[i].CMsg);
		}		
	}
	
	// 2. send SYNC msg and start synchronous motion 
 	SendSYNC();

	// 3. send 1 SDO form que

	// 4. evaluate recieve buffer
	while ((can_itf->receiveMsg(&cMsg)!=0) || (Msg_Counter>=MAX_InBuffer_Lenght))
	{	
		InBuffer.push(cMsg);
		Msg_Counter++;
	}
	//std::cout << "SYNC ID: " << std::hex << CANOpenMasterObj->TxSDO << std::endl;
	
	// ??? nötig
	pthread_exit(0);
} 

/*---------------------------------------------------------------*/
// send SYNC msg
void SendSYNC()
{	
	CanMsg CMsgTr;
	
	CMsgTr.m_iLen = 0;
	CMsgTr.m_iID = CANObj.SYNC;

	can_itf->transmitMsg(CMsgTr);
	
	//if(can_itf->receiveMsg(&CMsgTr)!=0)
	//{CMsgTr.print();}
}
