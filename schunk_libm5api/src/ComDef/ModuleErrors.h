
/******************************************************************************
 * 
 * Copyright (c) 2012 
 * 
 * SCHUNK GmbH & Co. KG
 *  
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
 * 
 * Project name: Drivers for "Amtec M5 Protocol" Electronics V4
 *                                                                        
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
 * 
 * Email:robotics@schunk.com
 * 
 * ToDo: 
 * 
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 *  * Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of SCHUNK GmbH & Co. KG nor the names of its 
 *    contributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission. 
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
 ******************************************************************************/


#ifndef MODULEERRORS_H
#define MODULEERRORS_H

	// Module States
	
#define STATEID_MOD_ERROR						0x00000001L
#define STATEID_MOD_HOME						0x00000002L
#define STATEID_MOD_HALT						0x00000004L
#define STATEID_MOD_POWERFAULT					0x00000008L
#define STATEID_MOD_TOW_ERROR					0x00000010L
#define STATEID_MOD_COMM_ERROR					0x00000020L
#define STATEID_MOD_SWR							0x00000040L
#define STATEID_MOD_SW1							0x00000080L
#define STATEID_MOD_SW2							0x00000100L
#define STATEID_MOD_BRAKEACTIVE 				0x00000200L
#define STATEID_MOD_CURLIMIT					0x00000400L
#define STATEID_MOD_MOTION						0x00000800L
#define STATEID_MOD_RAMP_ACC					0x00001000L
#define STATEID_MOD_RAMP_STEADY					0x00002000L
#define STATEID_MOD_RAMP_DEC					0x00004000L
#define STATEID_MOD_RAMP_END					0x00008000L
#define STATEID_MOD_INPROGRESS					0x00010000L
#define STATEID_MOD_FULLBUFFER					0x00020000L
#define STATEID_MOD_POW_VOLT_ERR				0x00040000L
#define STATEID_MOD_POW_FET_TEMP				0x00080000L
#define STATEID_MOD_POW_WDG_TEMP				0x00100000L
#define STATEID_MOD_POW_SHORTCUR				0x00200000L
#define STATEID_MOD_POW_HALLERR					0x00400000L
#define STATEID_MOD_POW_INTEGRALERR				0x00800000L
#define STATEID_MOD_CPU_OVERLOAD				0x01000000L
#define STATEID_MOD_BEYOND_HARD				   	0x02000000L
#define STATEID_MOD_BEYOND_SOFT					0x04000000L

	// Module Config

#define CONFIGID_MOD_ENCODER_FEEDBACK			0x00000001L //OS: 25xx from 35xx in SETUP
#define CONFIGID_MOD_RESOLVER_FEEDBACK			0x00000002L //OS: 25xx from 35xx in SETUP
#define CONFIGID_MOD_ABSOLUTE_FEEDBACK			0x00000004L //OS: 25xx from 35xx in SETUP
#define CONFIGID_MOD_BRAKE_PRESENT				0x00000008L
#define CONFIGID_MOD_BRAKE_AT_POWERON			0x00000010L
#define CONFIGID_MOD_SWR_WITH_ENCODERZERO		0x00000020L
#define CONFIGID_MOD_SWR_AT_FALLING_EDGE		0x00000040L
#define CONFIGID_MOD_CHANGE_SWR_TO_LIMIT		0x00000080L
#define CONFIGID_MOD_SWR_ENABLED        		0x00000100L
#define CONFIGID_MOD_SWR_LOW_ACTIVE				0x00000200L
#define CONFIGID_MOD_SWR_EQ_POSLIMIT			0x00000400L //OS: 25xx
#define CONFIGID_MOD_SWR_USE_EXTERNAL			0x00000400L //OS: 35xx
#define CONFIGID_MOD_SW1_ENABLED				0x00000800L
#define CONFIGID_MOD_SW1_LOW_ACTIVE				0x00001000L
#define CONFIGID_MOD_SW1_EQ_POSLIMIT			0x00002000L //OS: 25xx
#define CONFIGID_MOD_SW1_USE_EXTERNAL			0x00002000L //OS: 35xx
#define CONFIGID_MOD_SW2_ENABLED				0x00004000L
#define CONFIGID_MOD_SW2_LOW_ACTIVE				0x00008000L
#define CONFIGID_MOD_SW2_EQ_POSLIMIT			0x00010000L //OS: 25xx
#define CONFIGID_MOD_SW2_USE_EXTERNAL			0x00010000L //OS: 35xx
#define CONFIGID_MOD_LINEAR						0x00020000L
#define CONFIGID_MOD_OPENBRAKE_ON_SW2			0x00040000L	//OS: 25xx but not in 2512 from 35xx in SETUP
#define CONFIGID_MOD_ALLOW_FULL_CUR				0x00080000L
#define CONFIGID_MOD_M3_COMPATIBLE				0x00100000L
#define CONFIGID_MOD_LINEAR_SCREW				0x00200000L
#define CONFIGID_MOD_DISABLE_ON_HALT			0x00800000L
#define CONFIGID_MOD_WATCHDOG_ENABLE			0x01000000L
#define CONFIGID_MOD_ZERO_MOVE_AFTER_HOK		0x02000000L //OS: 35xx
#define CONFIGID_MOD_DISABLE_ACK				0x04000000L
#define CONFIGID_MOD_SYNC_MOTION				0x08000000L
#define CONFIGID_MOD_SYNC_TRANSMIT				0x10000000L
#define CONFIGID_MOD_CAN_RS232_COMM				0x40000000L //OS: 25xx
#define CONFIGID_MOD_PROFI_RS485_COMM			0x80000000L //OS: 25xx

	// Module Setup

// ---- FEEDBACK SETUP -----------
#define SETUPID_MOD_ENCODER_FEEDBACK			0x00000001L
#define SETUPID_MOD_RESOLVER_FEEDBACK			0x00000002L
#define SETUPID_MOD_ABSOLUTE_FEEDBACK			0x00000004L
// ---- DIGITAL IO SETUP ---------
#define SETUPID_MOD_4IN_4OUT					0x00000008L
#define SETUPID_MOD_3IN_ENCODER_IN				0x00000010L
#define SETUPID_MOD_3IN_ENCODER_OUT				0x00000020L
// ---- COMM INTERFACE -----------
#define SETUPID_MOD_RS232						0x00000040L
#define SETUPID_MOD_CAN							0x00000200L
#define SETUPID_MOD_PROFIBUS					0x00000400L
// ---- CAN ID SETUP -------------
#define SETUPID_MOD_USE_M3ID					0x00000800L
#define SETUPID_MOD_USE_M4ID					0x00001000L
#define SETUPID_MOD_USE_CANOPEN					0x00002000L
// ---- MOTOR SETUP --------------
#define SETUPID_MOD_INVERT_MOTORDIR				0x00004000L
// ---- INPUT USAGE --------------
#define SETUPID_MOD_USE_SW2_AS_ENABLE			0x00008000L
#define SETUPID_MOD_USE_SW2_AS_BRAKE			0x00010000L
// ---- OUTPUT USAGE -------------
#define SETUPID_MOD_ERROR_TO_OUT0				0x00020000L
// ---- IN/OUTPUT USAGE -------------
#define SETUPID_MOD_IO_GRIPPER_CONTROL			0x00080000L

	// Module Types

#define	TYPEID_MOD_ROTARY						0x0f
#define TYPEID_MOD_LINEAR						0xf0

	// Module DIO's

#define DIOID_MOD_INBIT0						0x00000001L
#define DIOID_MOD_INBIT1						0x00000002L
#define DIOID_MOD_INBIT2						0x00000004L
#define DIOID_MOD_INBIT3						0x00000008L
 
#define DIOID_MOD_OUTBIT0						0x00000010L
#define DIOID_MOD_OUTBIT1						0x00000020L
#define DIOID_MOD_OUTBIT2						0x00000040L
#define DIOID_MOD_OUTBIT3						0x00000080L
 
#define DIOID_MOD_INSWR							0x00000100L
#define DIOID_MOD_INSW1							0x00000200L
#define DIOID_MOD_INSW2							0x00000400L

	// Module Baudrates

#define BAUDRATEID_MOD_CAN_125K					0
#define BAUDRATEID_MOD_CAN_250K					1
#define BAUDRATEID_MOD_CAN_500K					2
#define BAUDRATEID_MOD_CAN_1000K				3

#define BAUDRATEID_MOD_RS232_1200				0
#define BAUDRATEID_MOD_RS232_2400				1
#define BAUDRATEID_MOD_RS232_4800				2
#define BAUDRATEID_MOD_RS232_9600				3
#define BAUDRATEID_MOD_RS232_19200				4
#define BAUDRATEID_MOD_RS232_38400				5
#define BAUDRATEID_MOD_RS232_56700				6

	// Module Errors
	
#define	ERRID_MOD					(-300L)		// module error
#define ERRID_MOD_INITERROR			(ERRID_MOD - 1)
#define ERRID_MOD_ISINITIALIZED		(ERRID_MOD - 2)
#define ERRID_MOD_NOTINITIALIZED	(ERRID_MOD - 3)
#define ERRID_MOD_WRONGMODULEID		(ERRID_MOD - 4)
#define ERRID_MOD_DEVICENOTFOUND	(ERRID_MOD - 5)
#define ERRID_MOD_DEVICENOTINIT		(ERRID_MOD - 6)
#define ERRID_MOD_MODULEERROR		(ERRID_MOD - 7)
#define ERRID_MOD_WAITTIMEOUT		(ERRID_MOD - 8)
#define ERRID_MOD_CONNECTIONNOTINIT	(ERRID_MOD - 9)
#define ERRID_MOD_PARSERERROR		(ERRID_MOD - 10)
#endif
