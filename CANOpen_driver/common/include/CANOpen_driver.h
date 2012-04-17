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

#ifndef CANOPENMaster_INCLUDEDEF_H
#define CANOPENMaster_INCLUDEDEF_H
//-----------------------------------------------
#include <ros/ros.h>
#include <cob_generic_can/CanItf.h>
#include <libpcan/libpcan.h>
#include <cob_utilities/IniFile.h>
#include <cob_generic_can/CanPeakSysUSB.h>

// Object definitions
#include <CANOpenCiA401_Objects.h>
//-----------------------------------------------

class CANOpenMaster
{
public:
	// --------------- Interface
	CANOpenMaster();
	~CANOpenMaster();

	int Init();
	int Homing();
	int GetNodeState();
	void PrintMotorStatus();
	int SetSpeed(int); 
	int GetSpeed();
	int GetDIn();
	int Recover(); 
	void SendSYNC();
	int WritePDO(int ,int);
	int WritePDO(CanMsg);

	// --------------- Types
public:	
	int m_CanBaseAddress;
	
	int TxSDO;
	int RxSDO;
	int TxPDO1;
	int RxPDO1;
	int TxPDO2;
	int RxPDO2;
	int TxPDO3;
	int RxPDO3;
	int TxPDO4;
	int RxPDO4;
	int SYNC; 
	int EMCY; 
	int NMT; 
	int HEARTBEAT; 
	int NodeState;
	int ControlerState; 
	
	// motorcommands
	short SWITCH_ON; 
	short SHUTDOWN; 
	short DISABLE_VOLTAGE; 
	short QUICKSTOP;
	short DISABLE_OPERATION;
	short ENABLE_OPERATION; 
	short FAULT_RESET;  
	
	// motorstates
	short Not_Ready_To_Switch_On;
	short Switch_On_Disabled;
	short Ready_To_Switch_On; 
	short Switched_On; 
	short Operation_Enable; 
	short Fault; 
	short Fault_Reaction_Active;
	short Quick_Stop_Active; 	
	
	unsigned char CANDownloadHeader;
	
	// statusvariables for state mashine
	bool IS_HOMED;
	bool FAULT_WAS_ACTIVE;
	
private:
	int Lin_Axis_max_speed;

	CanItf* can_itf;
	CANOpenCiA401ObjDirectory* CANObj; 	
	// --------------- Methodes
	
	// basic CANOpen functions
	void SYNCLoop(void*);
	bool CANError(CanMsg*);
	bool NodeStateError(CanMsg*);
	unsigned int ReadObject(CANOpenCiA401ObjDirectory::CANOpenObj*);
	int WriteObject(CANOpenCiA401ObjDirectory::CANOpenObj*, int);
	void SendNMT(unsigned char, unsigned char);
	int WaitForHeartbeat();
	int EvaluateControlerState();
	
};
//-----------------------------------------------

#endif

