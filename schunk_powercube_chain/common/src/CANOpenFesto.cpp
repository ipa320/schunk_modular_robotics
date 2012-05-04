/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: 
 * ROS stack name: 
 * ROS package name: 
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: 
 * Supervised by: 
 *
 * Date of creation: Jan 2010
 * ToDo:
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

//##################
//#### includes ####

// header
#include <CANOpenFesto.h>

// standard includes
#include <stdlib.h>
 

/* ============================================================== */    
/* ======================= public =============================== */    
/* ============================================================== */    

// Constructor
CANOpenFesto::CANOpenFesto()
{		
	can_itf = new CANPeakSysUSB("/home/cin-reha/git/LinearAxis/CanCtrl.ini");       
	CANObj = new CANOpenCiA401ObjDirectory();
	
    int m_CanBaseAddress = 2;

	TxSDO = 0x580 + m_CanBaseAddress;
	RxSDO = 0x600 + m_CanBaseAddress;
	SYNC = 0x080; 
	EMCY = 0x080 + m_CanBaseAddress; 
	NMT = 0x000; 
	HEARTBEAT = 0x700 + m_CanBaseAddress; 
	
	NodeState = 0x05; 
	ControlerState = 0; 
	
	// motorcommands
	SWITCH_ON = 0x07; 
	SHUTDOWN = 0x06; 
	DISABLE_VOLTAGE = 0x00; 
	QUICKSTOP = 0x02;
	DISABLE_OPERATION = 0x07;
	ENABLE_OPERATION = 0x0F; 
	FAULT_RESET = 0x0080;  
	
	// motorstates
	Not_Ready_To_Switch_On = 0x0000;
	Switch_On_Disabled = 0x0040;
	Ready_To_Switch_On = 0x0021;
	Switched_On = 0x0023; 
	Operation_Enable = 0x0027; 
	Fault = 0x0008; 
	Fault_Reaction_Active = 0x000F;
	Quick_Stop_Active = 0x0007; 	
	
	// statusvariables for state mashine
	IS_HOMED = false; 
	FAULT_WAS_ACTIVE = true; 
	
	// variabels for CAN-msg download header
	const int ciInitDownloadReq = 0x20;
	const int ciNrBytesNoData = 0x00;
	const int ciExpedited = 0x02;
	const int ciDataSizeInd = 0x01;

	CANDownloadHeader = ciInitDownloadReq | (ciNrBytesNoData << 2) | ciExpedited | ciDataSizeInd;

	Lin_Axis_max_speed = 83; //m/s
}

/*---------------------------------------------------------------*/
// Destructor
CANOpenFesto::~CANOpenFesto() 
{
	delete can_itf; 
	delete CANObj;
}

/*---------------------------------------------------------------*/
// Inits the CanOpen interface to the linear axis
int CANOpenFesto::Init()
{
	int nErr = 0, hb=99, timer = 0, timeout = 1000; 
	
	// reset all nodes
	SendNMT(0x00,0x81); 	
	
	// wait for inital heardbeat
	while (timer<=timeout){if (WaitForHeartbeat()==0){break;} timer++;}
	if (timer==timeout){ROS_INFO("timeout on initial heardbeat of linear axis");  
		return -1;
	} 
	
	//initiate CAN-node 
	SendNMT(0x02,0x01); 	

	usleep(500000);
	
	// set producer Heartbeat 
	hb=WriteObject(&CANObj->producer_heartbeat_time, 0x1388); // 0xEA60 == 60sec
	
	// wait for correct (0x05) nodestate
	while (timer<=timeout){if (WaitForHeartbeat()==0x05){break;} timer++;}
	if (timer==timeout){ROS_INFO("timeout on node init of linear axis");  
		return -1;
	} 
	
	// init procedure
		
		// read errors 
		try{
		nErr = (0x0000FF00 & ReadObject(&CANObj->pre_defined_error_field[0])); 
		}
		catch(int error) 
		{	std::cout << "Error on reading: " << error << std::endl;}
		if (nErr != 0) 
		{	std::cout << "There are Errors: " << std::endl; 
			for (int i=0;i<nErr;i++)
			{ 	try{std::cout << "Error" << i <<": " << ReadObject(&CANObj->pre_defined_error_field[i]) << std::endl;} 
				catch(int error) 
				{std::cout << "Error on reading: " << error << std::endl; return -1;}
			}
		}
		
		// reset faults
		if (WriteObject(&CANObj->controlword, FAULT_RESET)!=0){return -1;}
	
	FAULT_WAS_ACTIVE = false; 
	
	return 0; 
}

/*---------------------------------------------------------------*/
// home axis
int CANOpenFesto::Homing()
{	
	int timer = 0; 
	int referenced = 0; 

	try{std::cout << "mode: " << ReadObject(&CANObj->mode_of_operation_display) << std::endl;}
			catch(int error) 
			{ROS_INFO("Error on reading: %i",	error);return -1;}
	
	usleep(500000);
	
	// set mode of operation to homing
	if (WriteObject(&CANObj->mode_of_operation, CANObj->HOMING_MODE)!=0){return -2;}
	
	// wait for activating homing mode 
	while(timer<500)
	{	
		try{
			referenced = ReadObject(&CANObj->mode_of_operation_display); 
			}
			catch(int error) 
			{ROS_INFO("Error on reading: %i",	error);return -3;}
		
		if(referenced == CANObj->HOMING_MODE){break;}
		
		timer++;
		usleep(1000);
	}
			
	// start homing with sending controlword
	if (WriteObject(&CANObj->controlword, (0x0010 | ENABLE_OPERATION))!=0){return -4;}

	timer = 0; 
	// wait for referencing bit in manufacturer status word indication finished homing
	while(timer<60)
	{	
		try
		{
			referenced = (ReadObject(&CANObj->manufacturer_status_register)); 
		}
		catch(int error) 
		{
			ROS_INFO("Error on reading: %i",	error);return -5;
		}
		
		if((referenced & 0x00100000)== 0x00100000)
		{	
			// set mode of operation to velocity mode
			if (WriteObject(&CANObj->mode_of_operation, CANObj->VELOCITY_MODE)!=0){return -6;}
			usleep(50000);
			if (ReadObject(&CANObj->mode_of_operation_display) != CANObj->VELOCITY_MODE){return -7;} 
			IS_HOMED = true; 
			return 0;
		}
		
		timer++;
		usleep(1000000);
	}

	return -8; 
	
}

/*---------------------------------------------------------------*/
// set speed
int CANOpenFesto::SetSpeed(int SpeedCmdMS)
{	// Limit speed
	if (SpeedCmdMS > Lin_Axis_max_speed){SpeedCmdMS = Lin_Axis_max_speed;}
	if (SpeedCmdMS < -Lin_Axis_max_speed){SpeedCmdMS = -Lin_Axis_max_speed;}
	
	if (WriteObject(&CANObj->target_velocity, SpeedCmdMS)!=0){return -1;}
	return 0;
}

/*---------------------------------------------------------------*/
// get speed
int CANOpenFesto::GetSpeed()
{	
	int SpeedMS; 
	try{SpeedMS = ReadObject(&CANObj->velocity_actual_value);}
	catch(int error){ROS_INFO("Error on reading: %i",error);return -1;}
	return SpeedMS;
}

/*---------------------------------------------------------------*/
// get digital inputs
int CANOpenFesto::GetDIn()
{	
	int DIn; 
	try{DIn = ReadObject(&CANObj->digital_inputs);}
	catch(int error){ROS_INFO("Error on reading: %i",error);return -1;}
	return DIn;
}

/*---------------------------------------------------------------*/
// recover from errors
int CANOpenFesto::Recover()
{	
	int nErr = 0, timer=0; 
	// read status
	EvaluateControlerState();
	
	while((ControlerState == Fault) || (ControlerState == Fault_Reaction_Active) || (ControlerState == Quick_Stop_Active))
	{	
		// reset faults (positive edge of bit 7)
		if (WriteObject(&CANObj->controlword, 0x00)!=0){return -1;}
		usleep(500000);
	
		if (WriteObject(&CANObj->controlword, FAULT_RESET)!=0){return -1;}
		usleep(500000);
	
		// read status
		EvaluateControlerState();
		
		// timeout watchdog
		if (timer>10)
		{return -1;}
		else 
		{timer++;}
	}
		
	if (ControlerState != Switch_On_Disabled)
	{ROS_INFO("error on transition to Switch_On_Disabled.Controllerstate = %i",ControlerState); 
		try{
		nErr = (0x0000FF00 & ReadObject(&CANObj->pre_defined_error_field[0])); 
		}
		catch(int error) 
		{	std::cout << "Error on reading: " << error << std::endl;}
		if (nErr != 0) 
		{	std::cout << "There are Errors: " << std::endl; 
			for (int i=0;i<nErr;i++)
			{ 	try{std::cout << "Error" << i <<": " << ReadObject(&CANObj->pre_defined_error_field[i]) << std::endl;} 
				catch(int error) 
				{std::cout << "Error on reading: " << error << std::endl; return -1;}
			}
		}else{std::cout << "No Errors saved" << std::endl;}
		return -1;
	}
	
	usleep(100000);
	
	// go to ready_to_switch_on
	if (WriteObject(&CANObj->controlword, SHUTDOWN)!=0){return -1;}
	
	usleep(100000);
	
	// read status
	if(EvaluateControlerState()!=0){return -1;} 
	if (ControlerState != Ready_To_Switch_On)
	{ROS_INFO("error on transition to Ready_To_Switch_On. Controllerstate = %i",ControlerState); return -1;}
	
	usleep(100000);
		
	// go to Switched_On
	WriteObject(&CANObj->controlword, SWITCH_ON);
	
	usleep(100000);
	
	// read status
	if(EvaluateControlerState()!=0){return -1;} 
	if (ControlerState != Switched_On)
	{ROS_INFO("error on transition to Switched_On.Controllerstate = %i",ControlerState); return -1;}
	
	usleep(200000);
		
	// go to Operation_Enable
	WriteObject(&CANObj->controlword, ENABLE_OPERATION);
	
	usleep(500000);
	
	// read status
	EvaluateControlerState(); 
	if (ControlerState != Operation_Enable)
	{ROS_INFO("error on transition to Operation_Enable."); return -1;}
	
	FAULT_WAS_ACTIVE = false; 
	
	return 0; 
}

/*---------------------------------------------------------------*/
// get Nodestate
int CANOpenFesto::GetNodeState()
{	
	unsigned int status; 
	
	try{status = ReadObject(&CANObj->manufacturer_status_register);} 
	catch(int error){ std::cout << "Error: " << error << std::endl; return -1;}
	
	// print state	
		// Bit 0 
		if ((status & 1)==1)
			{	std::cout << "Homing active" << std::endl; }
		// Bit 1 
		if ((status & 2)==2)
			{	std::cout << "Homingswitch reached" << std::endl; }
		// Bit 2 
		if ((status & 4)==4)
			{	std::cout << "negative endswitch reached" << std::endl; }
		// Bit 3 
		if ((status & 8)==8)
			{	std::cout << "positive endswitch reached" << std::endl; }
		// Bit 4 
		if ((status & 16)==16)
			{	std::cout << "positioning successfull" << std::endl; }		
		// Bit 5 
		if ((status & 32)==32)
			{	std::cout << "target reached" << std::endl; }		
		// Bit 6 
		if ((status & 64)==64)
			{	std::cout << "rest of positioning distance reached" << std::endl; }	
		// Bit 7 
		if ((status & 128)==128)
			{	std::cout << "reversive mode" << std::endl; }	
		// Bit 8 
		if ((status & 256)==256)
			{	std::cout << "rpm reached n_ist=(n_mel +/-n_mel_hyst)" << std::endl; }		
		// Bit 9 
		if ((status & 512)==512)
			{	std::cout << "rpm reached n_ist=(n_soll +/-n_mel_hyst)" << std::endl; }	
		// Bit 10 
		if ((status & 1024)==1024)
			{	std::cout << "Positioning started" << std::endl; }	
		// Bit 11 
		if ((status & 2048)==2048)
			{	std::cout << "I²t, Current-limiting active" << std::endl; }	
		// Bit 12 
		if ((status & 4096)==4096)
			{	std::cout << "SinCos producer active" << std::endl; }	
		// Bit 13 
		if ((status & 8192)==8192)
			{	std::cout << "rpm reached n_ist=(0 +/-n_mel_hyst)" << std::endl; }	
		// Bit 14 
		if ((status & 16384)==16384)
			{	std::cout << "rpm reached n_ist=(0 +/-n_mel_hyst)" << std::endl; }	
		// Bit 15 
		if ((status & 32768)==32768)
			{	std::cout << "rpm reached n_ist=(0 +/-n_mel_hyst)" << std::endl; }	
		// Bit 16
		if ((status & 65536)==65536)
			{	std::cout << "Warning" << std::endl; }	
		// Bit 17
		if ((status & 131072)==131072)
			{	std::cout << "General Error present" << std::endl; }		
		// Bit 18
		if ((status & 262144)==262144)
			{	std::cout << "negative direction resticted" << std::endl; }			
		// Bit 19
		if ((status & 524288)==524288)
			{	std::cout << "positiv direction resticted" << std::endl; }	
		// Bit 20
		if ((status & 1048576)==1048576)
			{	std::cout << "Homeing done" << std::endl; }	
		// Bit 21
		if ((status & 2097152)==2097152)
			{	std::cout << "automatic syncronization activ" << std::endl; }	
		// Bit 22
		if ((status & 4194304)==4194304)
			{	std::cout << "MMC initiated" << std::endl; }	
		// Bit 23
		if ((status & 8388608)==8388608)
			{	std::cout << "Amps enabled" << std::endl; }	
		// Bit 24
		if ((status & 16777216)==16777216)
			{	std::cout << "controler and amps enabled" << std::endl; }	
		// Bit 25
		if ((status & 33554432)==33554432)
			{	std::cout << "rpm desired value enabled" << std::endl; }				
		// Bit 26
		if ((status & 67108864)==67108864)
			{	std::cout << "emergency stop without positioinsensor" << std::endl; }		
		// Bit 27
		if ((status & 134217728)==134217728)
			{	std::cout << "MOTID-Mode" << std::endl; }	
		// Bit 28
		if ((status & 268435456)==268435456)
			{	std::cout << "write permission" << std::endl; }
		// Bit 29
		if ((status & 536870912)==536870912)
			{	std::cout << "Technologiemodul present" << std::endl; }
		// Bit 30
		if ((status & 1073741824)==1073741824)
			{	std::cout << "MMC pluged in" << std::endl; }
		// Bit 31
		if ((status & (unsigned int) 2147483648)== (unsigned int) 2147483648)
			{	std::cout << "safe stop pluged in" << std::endl; }
		
	return 0; 
}

/*---------------------------------------------------------------*/
void CANOpenFesto::PrintMotorStatus()
{
	CanMsg CMsgTr;
	const int ciInitUploadReq = 0x40;

	CMsgTr.m_iLen = 6;
	CMsgTr.m_iID = RxSDO;

	unsigned char cMsg[8];

	cMsg[0] = ciInitUploadReq;
	cMsg[1] = 0x41;
	cMsg[2] = 0x60;
	cMsg[3] = 0x00;
	cMsg[4] = 0x00;
	cMsg[5] = 0x00;
	cMsg[6] = 0x00;
	cMsg[7] = 0x00;

	CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
	can_itf->transmitMsg(CMsgTr);

	while (can_itf->receiveMsg(&CMsgTr)==0)
	{	
		//std::cout << "waiting for statusreport" << std::endl;
		CMsgTr.print();
		usleep(500);
	}
	CMsgTr.print();
	if (CMsgTr.getAt(0) == 0x80)
	{	std::cout << "Error on status request. SDO Errorcode: " << std::hex << CMsgTr.getAt(7) << " " << std::hex << CMsgTr.getAt(6) << " " << std::hex << CMsgTr.getAt(5) << " " << std::hex << CMsgTr.getAt(4) << std::endl;
	}
	else
	{
		short CurrState = ((CMsgTr.getAt(7)<<8)|CMsgTr.getAt(6)); 
		//std::cout << CurrState << std::endl;

		// bits 0-3 and 5-6 displays state of the motorcontoller

			// Not_Ready_To_Switch_On (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==0)
			{	std::cout << "MotorState: Not_Ready_To_Switch_On" << std::endl; }
	
			// Switch_On_Disabled (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==0x0040)
			{	std::cout << "MotorState: Switch_On_Disabled" << std::endl; }
	
			// Ready_To_Switch_On 
			if ((CurrState & 0x006F)==0x0021)
			{	std::cout << "MotorState: Ready_To_Switch_On" << std::endl; }
	
			// Switched_On 
			if ((CurrState & 0x006F)==0x0023)
			{	std::cout << "MotorState: Switched_On" << std::endl; }
	
			// Operation_Enable 
			if ((CurrState & 0x006F)==0x0027)
			{	std::cout << "MotorState: Operation_Enable" << std::endl; }
	
			// Fault (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==0x0008)
			{	std::cout << "MotorState: Fault" << std::endl; }

			// Fault_Reaction_Active (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==0x00F)
			{	std::cout << "MotorState: Fault_Reaction_Active" << std::endl; }
	
			// Quick_Stop_Active
			if ((CurrState & 0x006F)==0x0007)
			{	std::cout << "MotorState: Quick_Stop_Active" << std::endl; }
	
		// bit 4 displays the voltage state
		if ((CurrState & 0x0010)==0x0010)
		{	std::cout << "voltage_enabled" << std::endl; } 

		// bit 7 displays if there are warnings 
		if ((CurrState & 0x0080)==0x0080)
		{	std::cout << "There are Warnings! Check!" << std::endl; } 

		// bit 8 is enabled if the drive moves 
		if ((CurrState & 0x0100)==0x0100)
		{	std::cout << "drive moves" << std::endl; } 

		// bit 9 is set when remote control via can is active
		if ((CurrState & 0x0200)==0x0200)
		{	std::cout << "voltage_enabled" << std::endl; } 

		// bit 10 is set if target position is reached 
		//(point in point mode, velocity in speed mode) 
		if ((CurrState & 0x0400)==0x0400)
		{	std::cout << "target reached" << std::endl; }

		// bit 11 is set when internal limits are reached 
		if ((CurrState & 0x0800)==0x0800)
		{	std::cout << "internal limits reached" << std::endl; } 

		// bit 12 set_point_acknowladge / speed_0 / homing_attained / ip_mode_active
		if ((CurrState & 0x1000)==0x1000)
		{	std::cout << "set_point_acknowladge / speed_0 / homing_attained / ip_mode_active" << std::endl; }  

		// bit 13 following_error / homing_error
		if ((CurrState & 0x2000)==0x2000)
		{	std::cout << "following_error / homing_error" << std::endl; } 

		// bit 14 reserved 

		// bit 15 drive referenced 
		if ((CurrState & 0x8000)==0x8000)
		{	std::cout << "drive referenced" << std::endl; } 
	}
}

/* ============================================================== */    
/* ====================== private =============================== */    
/* ============================================================== */    



/*---------------------------------------------------------------*/
// handle CAN Error 
bool CANOpenFesto::CANError(CanMsg* CMsgTr)
{	
	// error case for SDOs
	if (CMsgTr->getAt(0) == 0x80) 
	{
		ROS_INFO("Error on CAN request: %i %i %i %i", CMsgTr->getAt(4), CMsgTr->getAt(5), CMsgTr->getAt(6), CMsgTr->getAt(7));
		return true; 
	}
	
	return false;  
}
/*---------------------------------------------------------------*/
// Nodestate Error (test if heardbeat message has changed)
bool CANOpenFesto::NodeStateError(CanMsg* CMsgTr)
{	
	// if Msg is Heartbeat 
	if (CMsgTr->m_iID == HEARTBEAT)  
	{	
		//if Nodestate has changed send an error
		if (CMsgTr->getAt(0) != NodeState)
		{
			ROS_INFO("Error! CAN-Node Linear Axis not operational. State: %i", CMsgTr->getAt(0));
			return true;
		} 
		//get next msg because heartbeat is evaluated
		can_itf->receiveMsg(CMsgTr);
	}
	
	return false;  
}
/*---------------------------------------------------------------*/
// read object from device
unsigned int CANOpenFesto::ReadObject(CANOpenCiA401ObjDirectory::CANOpenObj* obj)
{	
	CanMsg CMsgTr;
	
	int i, result = 0, timer = 0; 
	
	switch (obj->Format)
	{	case 0x2F : CMsgTr.m_iLen = 5;
		case 0x2b : CMsgTr.m_iLen = 6;
		case 0x23 : CMsgTr.m_iLen = 8;
	}
	
	CMsgTr.m_iID = RxSDO;

	unsigned char cMsg[8];

	cMsg[0] = 0x40;	//for read always 0x40 
	cMsg[1] = obj->Index;		// low byte
	cMsg[2] = obj->Index >> 8;	// high byte
	cMsg[3] = obj->SubIndex;
	cMsg[4] = 0x00; 
	cMsg[5] = 0x00;
	cMsg[6] = 0x00;
	cMsg[7] = 0x00;

	CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
	
	if (can_itf->transmitMsg(CMsgTr) == 0)
	{throw -1; } 

	while(timer<1000)
	{		
		can_itf->receiveMsg(&CMsgTr);
		
		if (NodeStateError(&CMsgTr)) {throw -2;}
		
		if (CANError(&CMsgTr)) {throw -3;}
		 
		if((CMsgTr.getAt(0) == (obj->Format+0x20) ) && (CMsgTr.getAt(1)==cMsg[1]) && (CMsgTr.getAt(2)==cMsg[2]) && (CMsgTr.getAt(3)==cMsg[3]))
		{
			for(i=4;i<CMsgTr.m_iLen;i++)
			{
				result += (CMsgTr.getAt(i) << (8*(i-4))); 
			} 
			return  result;  
		}
		timer++;
		usleep(1000);
	}
	CMsgTr.print();
	throw -4; 
}

/*---------------------------------------------------------------*/
// write object on device
int CANOpenFesto::WriteObject(CANOpenCiA401ObjDirectory::CANOpenObj* obj, int val)
{	
	CanMsg CMsgTr;
	
	int timer = 0; 
	
	switch (obj->Format)
	{	case 0x2F : CMsgTr.m_iLen = 5;
		case 0x2b : CMsgTr.m_iLen = 6;
		case 0x23 : CMsgTr.m_iLen = 8;
	}
	
	CMsgTr.m_iID = RxSDO;

	unsigned char cMsg[8];

	cMsg[0] = obj->Format;	//for read always 0x40 
	cMsg[1] = obj->Index;		// low byte
	cMsg[2] = obj->Index >> 8;	// high byte
	cMsg[3] = obj->SubIndex;
	cMsg[4] = val; 
	if (CMsgTr.m_iLen > 5){cMsg[5] = val >> 8;}else{cMsg[5] = 0;}
	if (CMsgTr.m_iLen > 6){cMsg[6] = val >> 16; cMsg[7] = val >> 24;}else{cMsg[6] = 0,cMsg[7] = 0;}

	CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
	
	if (can_itf->transmitMsg(CMsgTr) == 0)
	{return -1; } 

	// wait for ACK
	while(timer<1000)
	{	
		can_itf->receiveMsg(&CMsgTr);
		
		if (NodeStateError(&CMsgTr)) {return -2;}
		
		if (CANError(&CMsgTr)) {return -3;}
		
		if((CMsgTr.getAt(0) == 0x60) && (CMsgTr.getAt(1)==cMsg[1]) && (CMsgTr.getAt(2)==cMsg[2]) && (CMsgTr.getAt(3)==cMsg[3]))
		{
			// ACK
			return  0;  
		}
		timer++;
		usleep(1000);
	}
	CMsgTr.print(); 
	return -4; 
}


/*---------------------------------------------------------------*/
// send CAN-Network-Management commands to control the state-machine of the CAN-node
void CANOpenFesto::SendNMT(unsigned char NodeID, unsigned char NMT_Command)
{
	CanMsg CMsgTr;

	CMsgTr.m_iLen = 2;
	CMsgTr.m_iID = 0x000;

	unsigned char cMsg[2];

	cMsg[0] = NMT_Command;
	cMsg[1] = NodeID; 

	CMsgTr.set(cMsg[0], cMsg[1]);
	can_itf->transmitMsg(CMsgTr);
}

/*---------------------------------------------------------------*/
	/** read Heardbeat
	 *  Waits for the Heartbeat from node. All messages that are recieved in between get lost! 
	 */ 
	int CANOpenFesto::WaitForHeartbeat()
	{
		CanMsg CMsgTr;
		int timer = 0; 
		
		while(timer<100)
		{	can_itf->receiveMsg(&CMsgTr);
			if(CMsgTr.m_iID == HEARTBEAT)
			{return CMsgTr.getAt(0);}
			timer++;
			usleep(10000);
		}
		return -1; 
	}

/*---------------------------------------------------------------*/
	// sets global variables of controller state
	int CANOpenFesto::EvaluateControlerState()
	{	
		int CurrState; 
		
		try {CurrState = ReadObject(&CANObj->statusword);}
		catch(int error){return error;}
		
		// bits 0-3 and 5-6 displays state of the motorcontoller

			// Not_Ready_To_Switch_On (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==Not_Ready_To_Switch_On)
			{	ControlerState = Not_Ready_To_Switch_On;
				std::cout << "MotorState: Not_Ready_To_Switch_On" << std::endl; }
	
			// Switch_On_Disabled (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==Switch_On_Disabled)
			{	ControlerState = Switch_On_Disabled;
				std::cout << "MotorState: Switch_On_Disabled" << std::endl; }
	
			// Ready_To_Switch_On 
			if ((CurrState & 0x006F)==Ready_To_Switch_On)
			{	ControlerState = Ready_To_Switch_On;
				std::cout << "MotorState: Ready_To_Switch_On" << std::endl; }
	
			// Switched_On 
			if ((CurrState & 0x006F)==Switched_On)
			{	ControlerState = Switched_On;
				std::cout << "MotorState: Switched_On" << std::endl; }
	
			// Operation_Enable 
			if ((CurrState & 0x006F)==Operation_Enable)
			{	ControlerState = Operation_Enable;
				std::cout << "MotorState: Operation_Enable" << std::endl; }
	
			// Fault (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==Fault)
			{	ControlerState = Fault;
				std::cout << "MotorState: Fault" << std::endl; }

			// Fault_Reaction_Active (bit 5 doesn't matter) 
			if ((CurrState & 0x004F)==Fault_Reaction_Active)
			{	ControlerState = Fault_Reaction_Active;
				std::cout << "MotorState: Fault_Reaction_Active" << std::endl; }
	
			// Quick_Stop_Active
			if ((CurrState & 0x006F)==Quick_Stop_Active)
			{	ControlerState = Quick_Stop_Active;
				std::cout << "MotorState: Quick_Stop_Active" << std::endl; }
	
		// bit 4 displays the voltage state
		if ((CurrState & 0x0010)==0x0010)
		{	std::cout << "voltage_enabled" << std::endl; } 

		// bit 7 displays if there are warnings 
		if ((CurrState & 0x0080)==0x0080)
		{	std::cout << "There are Warnings! Check!" << std::endl; } 

		// bit 8 is enabled if the drive moves 
		if ((CurrState & 0x0100)==0x0100)
		{	std::cout << "drive moves" << std::endl; } 

		// bit 9 is set when remote control via can is active
		if ((CurrState & 0x0200)==0x0200)
		{	std::cout << "voltage_enabled" << std::endl; } 

		// bit 10 is set if target position is reached 
		//(point in point mode, velocity in speed mode) 
		if ((CurrState & 0x0400)==0x0400)
		{	std::cout << "target reached" << std::endl; }

		// bit 11 is set when internal limits are reached 
		if ((CurrState & 0x0800)==0x0800)
		{	std::cout << "internal limits reached" << std::endl; } 

		// bit 12 set_point_acknowladge / speed_0 / homing_attained / ip_mode_active
		if ((CurrState & 0x1000)==0x1000)
		{	std::cout << "set_point_acknowladge / speed_0 / homing_attained / ip_mode_active" << std::endl; }  

		// bit 13 following_error / homing_error
		if ((CurrState & 0x2000)==0x2000)
		{	std::cout << "following_error / homing_error" << std::endl; } 

		// bit 14 reserved 

		// bit 15 drive referenced 
		if ((CurrState & 0x8000)==0x8000)
		{	std::cout << "drive referenced" << std::endl; } 
		
	return 0;
	}
