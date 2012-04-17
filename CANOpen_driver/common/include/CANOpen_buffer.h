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

#ifndef CANOpen_buffer_INCLUDEDEF_H
#define CANOpen_buffer_INCLUDEDEF_H
//-----------------------------------------------

#include <cob_generic_can/CanItf.h>

/*=============================================*/
 
class CANOpen_buffer
{	
	private:
	
	// TODO make struct size variable
	// global PDO buffer 
	
	struct Data
	{	
		int NodeID;
		CanMsg CMsg;
		bool active;
		int desired_SYNC_freq_HZ; //experimental
	}data[32];	//8 nodes with 4 PDOs
	

	public:

	// constructor
	// param CAN-ID offset for specifing PDO1, PDO2, SDO... 
	CANOpen_buffer()
	{	
		for (int i=0;i<32;i++)
		{	
			data[i].active = false; 	
		}	
	}
	
	// destructor
	~CANOpen_buffer()
	{}

	int RegMsg(int, int, int);
	bool IsActive(int);
	int SetMsg(int, char*);
	CanMsg* GetMsg(int number);
};  	

	// set Msg in buffer
	// TODO check params befor setting
	int CANOpen_buffer::RegMsg(int ID, int number, int des_freq)
	{	
		if (!data[number].active)
		{	
			data[number].CMsg.m_iLen = 8;
			data[number].CMsg.m_iID = ID; 			
			data[number].active = true;
			data[number].desired_SYNC_freq_HZ = des_freq;
		}
		else 
		{return -1;} 	
		return 0;
	}	
	
	// Is active
	// TODO: test is ID is in range
	bool CANOpen_buffer::IsActive(int ID)
	{	
		return data[ID].active;
	}	
	
	// set Msg in buffer
	// TODO check params befor setting
	int CANOpen_buffer::SetMsg(int number, char* Msg)
	{	
			data[number].CMsg.set(Msg[0], Msg[1], Msg[2], Msg[3], Msg[4], Msg[5], Msg[6], Msg[7]);
		return 0;
	}	
	
	// Get Msg in buffer
	CanMsg* CANOpen_buffer::GetMsg(int number)
	{	
		if(!data[number].active)
		{ return &data[number].CMsg;}
		return 0;
	}	

//-----------------------------------------------
#endif

