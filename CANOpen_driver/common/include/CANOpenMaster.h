/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: CIN-Reha
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

#ifndef CANOpenMaster_INCLUDEDEF_H
#define CANOpenMaster_INCLUDEDEF_H
//-----------------------------------------------

// std includes
#include <pthread.h>
#include <queue>

// ros includes
#include <cob_utilities/Mutex.h>
#include <cob_generic_can/CanItf.h>
#include <cob_generic_can/CanPeakSysUSB.h>

// own includes
//#include <CANOpen_driver.h>
#include <CANOpen_buffer.h>
#include <CANOpenCiA401_Schunk.h>


// TODO make this variable, attention to CANOpen_buffer!
#define MAX_PDOS 32 //8 nodes with 4 PDOs
#define MAX_InBuffer_Lenght 32

CANOpenCiA401_Schunk CANObj; 
CANOpen_buffer* PDO_buffer;
CANPeakSysUSB* can_itf; 
std::queue<CanMsg> InBuffer;


// as API to use
void InitMaster();
void StartMasterThread(double);
void AcitvatePDO(int, int, double);
void AssignValue2PDO(int, char*);
int GetRecievedMsgs(CanMsg* );

/*=============================================*/
//protected

struct Thread_data
{	
	double Cycle_Freq_HZ;
}TD_Master;

void *MasterThreadFunc(void*);
void *EvaluateBuffers(void*);
void SendSYNC();


 #endif //CANOpenMaster_INCLUDEDEF_H
