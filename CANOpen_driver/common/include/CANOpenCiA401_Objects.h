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

/*=============================================*/
 
typedef struct 
	{	
	unsigned short Index;
	unsigned char SubIndex; 
	unsigned char Format; //UInt8/Int8 == 0x2F; Uint16/Int16 == 0x2B; UInt32/Int32 == 0x23
	}CANOpenObj;

	#define Errorcue 5
	#define POD_config_options 2
	#define POD_num_mappable_obj 64

	// ID definitions
	int	TxSDO = 0x580;
	int	RxSDO = 0x600;
	int	TxPDO1 = 0x180;
	int	RxPDO1 = 0x200;
	int	TxPDO2 = 0x280;
	int	RxPDO2 = 0x300;
	int	TxPDO3 = 0x380;
	int	RxPDO3 = 0x400;
	int	TxPDO4 = 0x480;
	int	RxPDO4 = 0x500;	
	int	SYNC = 0x080; 
	int	EMCY = 0x080; 
	int	NMT = 0x000; 
	int	HEARTBEAT = 0x700; 	


	CANOpenObj manufacturer_status_register;
	CANOpenObj pre_defined_error_field[Errorcue]; 
	CANOpenObj COB_ID_SYNC;

	// SYNC producer or consumer must be added to the node ID
	unsigned int SYNC_PRODUCER; 
	unsigned int SYNC_CONSUMER; 
	CANOpenObj Communication_cycle_period;
	CANOpenObj COB_ID_EMCY;

	// do EMCY msgs exist on node? must be added to the node ID
	unsigned int EMCY_EXIST; 
	unsigned int EMCY_DONT_EXIS; 
	CANOpenObj producer_heartbeat_time; 

	CANOpenObj COB_ID_TPDO1[POD_config_options];
	CANOpenObj COB_ID_RPDO1[POD_config_options];	
	CANOpenObj PDO_MAP_TPDO1[POD_num_mappable_obj];
	CANOpenObj PDO_MAP_RPDO1[POD_num_mappable_obj];

	CANOpenObj COB_ID_TPDO2[POD_config_options];
	CANOpenObj COB_ID_RPDO2[POD_config_options];
	CANOpenObj PDO_MAP_TPDO2[POD_num_mappable_obj];
	CANOpenObj PDO_MAP_RPDO2[POD_num_mappable_obj];

	CANOpenObj COB_ID_TPDO3[POD_config_options];
	CANOpenObj COB_ID_RPDO3[POD_config_options];
	CANOpenObj COB_ID_TPDO4[POD_config_options];		
	CANOpenObj COB_ID_RPDO4[POD_config_options];

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
	// Schunk specific
	CANOpenObj ip_sync_definition[2];
	CANOpenObj ip_data_configuration[6];
	CANOpenObj target_velocity; 

	CANOpenObj digital_inputs; 

	// 0x1000er mandatory objects	
		manufacturer_status_register.Index = 0x1002;
		manufacturer_status_register.SubIndex = 0x00;
		manufacturer_status_register.Format = 0x23;
/*
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
	
		COB_ID_SYNC.Index = 0x1005;
		COB_ID_SYNC.SubIndex = 0x00;
		COB_ID_SYNC.Format = 0x23;
			SYNC_PRODUCER = 0x40000080; 
			SYNC_CONSUMER = 0x00000080; 
	
		Communication_cycle_period.Index = 0x1006;
		Communication_cycle_period.SubIndex = 0x00;			
		Communication_cycle_period.Format = 0x23;

		COB_ID_EMCY.Index = 0x1014;
		COB_ID_EMCY.SubIndex = 0x00;
		COB_ID_EMCY.Format = 0x23;
			EMCY_EXIST = 0x00000080; //only for 11-bit CAN-base frame (29-bit frame = 0x20000080)
			EMCY_DONT_EXIS = 0x80000080; //only for 11-bit CAN-base frame (29-bit frame = 0xA0000080)

		producer_heartbeat_time.Index = 0x1017;
		producer_heartbeat_time.SubIndex = 0x00;
		producer_heartbeat_time.Format = 0x2b;

	//PDO CH 1	
		COB_ID_TPDO1[0].Index = 0x1800;	
		COB_ID_TPDO1[0].SubIndex = 0x00;
		COB_ID_TPDO1[0].Format = 0x23;

		COB_ID_TPDO1[1].Index = 0x1800;
		COB_ID_TPDO1[1].SubIndex = 0x01;
		COB_ID_TPDO1[1].Format = 0x23;
	
		COB_ID_TPDO1[2].Index = 0x1800;
		COB_ID_TPDO1[2].SubIndex = 0x02;
		COB_ID_TPDO1[2].Format = 0x2f;
	
		COB_ID_RPDO1[0].Index = 0x1400;	
		COB_ID_RPDO1[0].SubIndex = 0x00;
		COB_ID_RPDO1[0].Format = 0x23;

		COB_ID_RPDO1[1].Index = 0x1400;
		COB_ID_RPDO1[1].SubIndex = 0x01;
		COB_ID_RPDO1[1].Format = 0x23;
	
		COB_ID_RPDO1[2].Index = 0x1400;
		COB_ID_RPDO1[2].SubIndex = 0x02;
		COB_ID_RPDO1[2].Format = 0x2f;
	
		PDO_MAP_TPDO1[0].Index = 0x1A00;
		PDO_MAP_TPDO1[0].SubIndex = 0x00;
		PDO_MAP_TPDO1[0].Format = 0x2f;	
			for (int index=1;index<POD_num_mappable_obj;index++)
			{
				PDO_MAP_TPDO1[index].Index = 0x1A00;
		 		PDO_MAP_TPDO1[index].SubIndex = index;
		 		PDO_MAP_TPDO1[index].Format = 0x23;	
			}
	
		for (int index=1;index<POD_num_mappable_obj;index++)
		{
			PDO_MAP_RPDO1[index].Index = 0x1600;
	 		PDO_MAP_RPDO1[index].SubIndex = index;
	 		PDO_MAP_RPDO1[index].Format = 0x23;	
		}

	//PDO CH 2
		COB_ID_TPDO2[0].Index = 0x1801;	
		COB_ID_TPDO2[0].SubIndex = 0x00;
		COB_ID_TPDO2[0].Format = 0x23;

		COB_ID_TPDO2[1].Index = 0x1801;
		COB_ID_TPDO2[1].SubIndex = 0x01;
		COB_ID_TPDO2[1].Format = 0x23;
	
		COB_ID_TPDO2[2].Index = 0x1801;
		COB_ID_TPDO2[2].SubIndex = 0x02;
		COB_ID_TPDO2[2].Format = 0x2f;
	
		COB_ID_RPDO2[0].Index = 0x1401;	
		COB_ID_RPDO2[0].SubIndex = 0x00;
		COB_ID_RPDO2[0].Format = 0x23;

		COB_ID_RPDO2[1].Index = 0x1401;
		COB_ID_RPDO2[1].SubIndex = 0x01;
		COB_ID_RPDO2[1].Format = 0x23;
	
		COB_ID_RPDO2[2].Index = 0x1401;
		COB_ID_RPDO2[2].SubIndex = 0x02;
		COB_ID_RPDO2[2].Format = 0x2f;	

		for (int index=0;index<POD_num_mappable_obj;index++)
		{
			PDO_MAP_TPDO2[index].Index = 0x1A01;
	 		PDO_MAP_TPDO2[index].SubIndex = index;
	 		PDO_MAP_TPDO2[index].Format = 0x23;	
		}
	
		for (int index=0;index<POD_num_mappable_obj;index++)
		{
			PDO_MAP_RPDO2[index].Index = 0x1601;
	 		PDO_MAP_RPDO2[index].SubIndex = index;
	 		PDO_MAP_RPDO2[index].Format = 0x23;	
		}

	//PDO CH 3
		COB_ID_TPDO3[0].Index = 0x1802;	
		COB_ID_TPDO3[0].SubIndex = 0x00;
		COB_ID_TPDO3[0].Format = 0x23;

		COB_ID_TPDO3[1].Index = 0x1802;
		COB_ID_TPDO3[1].SubIndex = 0x01;
		COB_ID_TPDO3[1].Format = 0x23;
	
		COB_ID_TPDO3[2].Index = 0x1802;
		COB_ID_TPDO3[2].SubIndex = 0x02;
		COB_ID_TPDO3[2].Format = 0x2f;
	
		COB_ID_RPDO3[0].Index = 0x1402;	
		COB_ID_RPDO3[0].SubIndex = 0x00;
		COB_ID_RPDO3[0].Format = 0x23;

		COB_ID_RPDO3[1].Index = 0x1402;
		COB_ID_RPDO3[1].SubIndex = 0x01;
		COB_ID_RPDO3[1].Format = 0x23;
	
		COB_ID_RPDO3[2].Index = 0x1402;
		COB_ID_RPDO3[2].SubIndex = 0x02;
		COB_ID_RPDO3[2].Format = 0x2f;		
	//PDO CH 4	
		COB_ID_TPDO4[0].Index = 0x1803;	
		COB_ID_TPDO4[0].SubIndex = 0x00;
		COB_ID_TPDO4[0].Format = 0x23;

		COB_ID_TPDO4[1].Index = 0x1803;
		COB_ID_TPDO4[1].SubIndex = 0x01;
		COB_ID_TPDO4[1].Format = 0x23;
	
		COB_ID_TPDO4[2].Index = 0x1803;
		COB_ID_TPDO4[2].SubIndex = 0x02;
		COB_ID_TPDO4[2].Format = 0x2f;
	
		COB_ID_RPDO4[0].Index = 0x1403;	
		COB_ID_RPDO4[0].SubIndex = 0x00;
		COB_ID_RPDO4[0].Format = 0x23;

		COB_ID_RPDO4[1].Index = 0x1403;
		COB_ID_RPDO4[1].SubIndex = 0x01;
		COB_ID_RPDO4[1].Format = 0x23;
	
		COB_ID_RPDO4[2].Index = 0x1403;
		COB_ID_RPDO4[2].SubIndex = 0x02;
		COB_ID_RPDO4[2].Format = 0x2f;	


	// 0x6000er objects
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
	
		ip_sync_definition[1].Index = 0x60C3;
		ip_sync_definition[1].SubIndex = 0x01;
		ip_sync_definition[1].Format = 0x2f;
	
		ip_sync_definition[2].Index = 0x60C3;
		ip_sync_definition[2].SubIndex = 0x02;
		ip_sync_definition[2].Format = 0x2f;
	
		ip_data_configuration[3].Index = 0x60C4;
		ip_data_configuration[3].SubIndex = 0x03;
		ip_data_configuration[3].Format = 0x2f;
	
		ip_data_configuration[5].Index = 0x60C4;
		ip_data_configuration[5].SubIndex = 0x05;
		ip_data_configuration[5].Format = 0x2f;
	
		target_velocity.Index = 0x60FF;
		target_velocity.SubIndex = 0x00;
		target_velocity.Format = 0x23;
	
		digital_inputs.Index = 0x60FD;
		digital_inputs.SubIndex = 0x00;
		digital_inputs.Format = 0x23;
*/
//-----------------------------------------------
#endif

