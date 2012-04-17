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

#ifndef CANOpenCiA401_Objects_INCLUDEDEF_H
#define CANOpenCiA401_Objects_INCLUDEDEF_H
//-----------------------------------------------

/*===============================================
 * Object structure	

struct CANOpenObj
{	
	unsigned short Index;
	unsigned char SubIndex; 
	unsigned char Format;
};

 *=============================================*/
 
class CANOpenCiA401ObjDirectory
{	
	public: 
	
	struct CANOpenObj
	{	
		unsigned short Index;
		unsigned char SubIndex; 
		unsigned char Format; //UInt8/Int8 == 0x2F; Uint16/Int16 == 0x2B; UInt32/Int32 == 0x23
	};
	
	#define Errorcue 5
	
	CANOpenObj manufacturer_status_register;
	CANOpenObj pre_defined_error_field[Errorcue]; 
	CANOpenObj producer_heartbeat_time; 

	
	CANOpenObj controlword;
	CANOpenObj statusword;
	
	CANOpenObj mode_of_operation;
		// modes of operation 
		unsigned char POSITION_MODE; 
		unsigned char VELOCITY_MODE; 
		unsigned char TORQUE_MODE; 
		unsigned char HOMING_MODE; 
		unsigned char IP_MODE; //interpolated mode
	
	CANOpenObj mode_of_operation_display;
	
	CANOpenObj velocity_actual_value; 
	CANOpenObj target_velocity; 
	
	CANOpenObj digital_inputs; 
	
	// constructor
	CANOpenCiA401ObjDirectory()
	{	
		// 0x1000er mandatory objects	 	 
	 		pre_defined_error_field[0].Index = 0x1003;
	 		pre_defined_error_field[0].SubIndex = 0x00;
	 		pre_defined_error_field[0].Format = 0x2f;
	 		
	 		pre_defined_error_field[1].Index = 0x1003;
	 		pre_defined_error_field[1].SubIndex = 0x01;
	 		pre_defined_error_field[1].Format = 0x23;
	 		
	 		pre_defined_error_field[2].Index = 0x1003;
	 		pre_defined_error_field[2].SubIndex = 0x02;
	 		pre_defined_error_field[2].Format = 0x23;
	 		
	 		pre_defined_error_field[3].Index = 0x1003;
	 		pre_defined_error_field[3].SubIndex = 0x03;
	 		pre_defined_error_field[3].Format = 0x23;
	 		
	 		pre_defined_error_field[4].Index = 0x1003;
	 		pre_defined_error_field[4].SubIndex = 0x04;
	 		pre_defined_error_field[4].Format = 0x23;
	 		
	 		manufacturer_status_register.Index = 0x1002;
	 		manufacturer_status_register.SubIndex = 0x00;
	 		manufacturer_status_register.Format = 0x23;
	 		
	 		producer_heartbeat_time.Index = 0x1017;
	 		producer_heartbeat_time.SubIndex = 0x00;
	 		producer_heartbeat_time.Format = 0x2b;
	
		// 0x6000er mandatory objects
	 		controlword.Index = 0x6040;
	 		controlword.SubIndex = 0x00;
	 		controlword.Format = 0x2b;
	 		
	 		statusword.Index = 0x6041;
	 		statusword.SubIndex = 0x00;
	 		statusword.Format = 0x2b;
	 		
	 		mode_of_operation.Index = 0x6060;
	 		mode_of_operation.SubIndex = 0x00;
	 		mode_of_operation.Format = 0x2f;	
	 		
	 		// modes of operation values
		 	POSITION_MODE = 0x1; 
			VELOCITY_MODE = 0x3; 
			TORQUE_MODE = 0x4; 
			HOMING_MODE = 0x6; 
			IP_MODE = 0x7;
			
	 		mode_of_operation_display.Index = 0x6061;
	 		mode_of_operation_display.SubIndex = 0x00;
	 		mode_of_operation_display.Format = 0x2f;	
	 	
	 		velocity_actual_value.Index = 0x606c;
	 		velocity_actual_value.SubIndex = 0x00;
	 		velocity_actual_value.Format = 0x23;
			
			target_velocity.Index = 0x60FF;
	 		target_velocity.SubIndex = 0x00;
	 		target_velocity.Format = 0x23;
	 		
	 		digital_inputs.Index = 0x60FD;
	 		digital_inputs.SubIndex = 0x00;
	 		digital_inputs.Format = 0x23;
	}
	
	// destructor
	~CANOpenCiA401ObjDirectory()
	{}
};  	
	
	

//-----------------------------------------------
#endif

