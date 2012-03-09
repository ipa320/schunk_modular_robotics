
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

#ifndef PROTOCOLCOMMANDS
#define PROTOCOLCOMMANDS

//***************************************************************************
//
//  Messages ID
//
//***************************************************************************

// 11Bit ID
#define MSGID_ACK				0x0a0
#define MSGID_GET				0x0c0
#define MSGID_SET				0x0e0
#define MSGID_STATE				0x060
#define MSGID_ALL				0x100

#define MSGID_DLR_INIT_0		0x42e
#define MSGID_DLR_INIT_1		0x42c
#define MSGID_DLR_INIT_2		0x42c
#define MSGID_DLR_INIT_ACK		0x42b
#define MSGID_DLR_DATA_GET		0x42d
#define MSGID_DLR_DATA_ACK		0x3c5

#define MSGID_SCHUNK_SEND		0x300
#define MSGID_SCHUNK_RECV		0x200
#define MAX_SCHUNK	63

#define MSGID_EMS_START_ID		0x500
#define MSGID_EMS_CONFIG_ACK    0x7e4
#define MSGID_EMS_CONFIG		0x7e5

#define MSGID_MP55_SEND			0x600
#define MSGID_MP55_RECV			0x580
#define MAX_MP55	127

//***************************************************************************
//
//  Command ID (byte 0)
//
//***************************************************************************

#define CMDID_RESET				0x00	// message all
#define CMDID_HOME				0x01	// message all
#define CMDID_HALT				0x02	// message all
#define CMDID_TOGGLEHOME		0x03
#define CMDID_INTERNAL			0x04
#define CMDID_WATCHDOG			0x07	// message all

#define CMDID_SETPARAM			0x08
#define CMDID_RECALCPID			0x09
#define CMDID_BAUDRATE			0x09	// message all
#define CMDID_GETPARAM			0x0a
#define CMDID_SETMOVE				0x0b

#define CMDID_SAVEPOS					0x0e	// message all
#define CMDID_STARTMOVE				0x0f	// message all
#define CMDID_SAVEPARAMS			0x10

#define CMDID_DLR_DATA_GET		0x44
#define CMDID_DLR_DATA_ACK0		0x00
#define CMDID_DLR_DATA_ACK1		0x41
#define CMDID_DLR_DATA_ACK2		0x42
#define CMDID_DLR_DATA_ACK3		0x83

#define CMDID_SCHUNK_GET_FTI	0x4c
#define CMDID_SCHUNK_GET_TEI	0x4e
#define CMDID_SCHUNK_GET_TRI	0x56
#define CMDID_SCHUNK_SET_BAUDRATE	0x72
#define CMDID_SCHUNK_SET_MSGID	0x74
#define CMDID_SCHUNK_SET_NULL	0x7a

#define CMDID_EMS_SET_MSGID		0x80
#define CMDID_EMS_SET_STARTBIT	0x81

//***************************************************************************
//
//  Parameter ID (Byte 1)
//
//***************************************************************************

#define PARID_DLR_DATA_GET		0x31

#define PARID_MOVE_FRAMP		0x04
#define PARID_MOVE_FSTEP		0x06
#define PARID_MOVE_FVEL			0x07
#define PARID_MOVE_FCUR			0x08
#define PARID_MOVE_IRAMP		0x09
#define PARID_MOVE_ISTEP		0x0b
#define PARID_MOVE_IVEL			0x0c
#define PARID_MOVE_ICUR			0x0d
#define PARID_MOVE_FRAMP_EXT	0x0e
#define PARID_MOVE_FSTEP_EXT	0x10
#define PARID_MOVE_FVEL_EXT		0x11
#define PARID_MOVE_FCUR_EXT		0x12
#define PARID_MOVE_IRAMP_EXT	0x13
#define PARID_MOVE_ISTEP_EXT	0x15
#define PARID_MOVE_IVEL_EXT		0x16
#define PARID_MOVE_ICUR_EXT		0x17

#define PARID_DEF_FHOMEOFFSET			0x00	// read
#define PARID_DEF_FGEARRATIO			0x01	// read
#define PARID_DEF_FLINEARRATIO			0x02	// read
#define PARID_DEF_FMINPOS				0x03	// read
#define PARID_DEF_FMAXPOS				0x04	// read
#define PARID_DEF_FMAXDELTAPOS			0x05	// read
#define PARID_DEF_FCUROFFSET			0x07	// read
#define PARID_DEF_FCURRATIO				0x08	// read
#define PARID_DEF_FMAXVEL				0x0a	// read
#define PARID_DEF_FMAXACC				0x0c	// read
#define PARID_DEF_FMAXCUR				0x0e	// read
#define PARID_DEF_FHOMEVEL				0x0f	// read
#define PARID_DEF_FHOMEACC				0x10	// read
#define PARID_DEF_DIODATA				0x19	// read
#define PARID_DEF_SERIALNO				0x1a	// read
#define PARID_DEF_CONFIG				0x1b	// read
#define PARID_DEF_INCPERTURN			0x1c	// read
#define PARID_DEF_VERSION				0x1d	// read
#define PARID_DEF_BRAKETIMEOUT			0x1f	// read
#define PARID_DEF_ADDRESS					0x20	// read
#define PARID_DEF_CANBAUDRATE			0x22	// read/write
#define PARID_DEF_RSBAUDRATE			0x23	// read/write
#define PARID_ACT_IPOSCOUNT				0x24	// read
#define PARID_ACT_DIODATA				0x26	// read/write
#define PARID_ACT_STATE					0x27	// read
#define PARID_ACT_IRAMPVEL				0x29	// write
#define PARID_ACT_IRAMPACC				0x2a	// write
#define PARID_ACT_IHOMEOFFSET			0x2c	// read/write
#define PARID_ACT_ICUR					0x35	// read
#define PARID_ACT_CONFIG				0x39	// read/write
#define PARID_ACT_FINCRATIO				0x3b	// read/write
#define PARID_ACT_FPOS					0x3c	// read
#define PARID_ACT_FDELTAPOS				0x3f	// read
#define PARID_ACT_FMAXDELTAPOS			0x40	// read/write
#define PARID_ACT_FVEL					0x41	// read
#define PARID_ACT_FIPOLVEL				0x42	// read
#define PARID_ACT_FMINPOS				0x45	// read/write
#define PARID_ACT_FMAXPOS				0x46	// read/write
#define PARID_ACT_FMAXVEL				0x48	// read/write
#define PARID_ACT_FMAXACC				0x4a	// read/write
#define PARID_ACT_FMAXCUR				0x4c	// read/write
#define PARID_ACT_FCUR					0x4d	// read
#define PARID_ACT_FRAMPVEL				0x4f	// write
#define PARID_ACT_FRAMPACC				0x50	// write
#define PARID_DEF_C0  					0x51	// read
#define PARID_DEF_DAMP					0x52	// read
#define PARID_DEF_A0					0x53	// read
#define PARID_ACT_C0  					0x54	// read/write
#define PARID_ACT_DAMP					0x55	// read/write
#define PARID_ACT_A0					0x56	// read/write
#define PARID_DEF_BURNCOUNT				0x57	// read
#define PARID_DEF_SETUP					0x58	// read
#define PARID_ACT_FHOMEOFFSET			0x59	// read/write
#define PARID_ACT_IPOS					0x5a	// read
#define PARID_ACT_IMAXDELTAPOS			0x5b	// read/write
#define PARID_ACT_IMINPOS				0x5c	// read/write
#define PARID_ACT_IMAXPOS				0x5d	// read/write
#define PARID_ACT_IMAXVEL				0x5e	// read/write
#define PARID_ACT_IMAXACC				0x5f	// read/write
#define PARID_ACT_IVEL					0x60	// read
#define PARID_ACT_IDELTAPOS				0x61	// read
#define PARID_ACT_FPOSSTATEDIO			0x62	// read
#define PARID_ACT_FSAVEPOS				0x63	// read
#define PARID_ACT_FHOMEVEL				0x64	// read
#define PARID_ACT_IHOMEVEL				0x65	// read
#define PARID_ACT_SYNCTIME				0x66	// read/write
#define PARID_ACT_LOADLIMIT				0x67	// read/write
#define PARID_ACT_MAXLOADGRADIENT	0x68	// read/write
#define PARID_ACT_LOADDELTATIME		0x69	// read/write

#define PARID_ACT_RAWMOTCUR				0x6C	// read
#define PARID_ACT_RAWMOTSUPPLY		0x6D	// read
#define PARID_ACT_RAWTEMP					0x6E	// read
#define PARID_ACT_RAWLOGICSUPPLY	0x6F	// read
#define PARID_ACT_FMOTCUR					0x70	// read
#define PARID_ACT_FMOTSUPPLY			0x71	// read
#define PARID_ACT_FTEMP						0x72	// read
#define PARID_ACT_FLOGICSUPPLY		0x73	// read

#define PARID_ACT_MINLOGIC					0x74
#define PARID_ACT_MAXLOGIC					0x75
#define PARID_ACT_MINMOTOR					0x76
#define PARID_ACT_MAXMOTOR					0x77
#define PARID_ACT_NOMCUR						0x78
#define PARID_ACT_HMAXCUR						0x79
#define PARID_ACT_LOGICUNDERSHOOT		0x7A
#define PARID_ACT_LOGICOVERSHOOT		0x7B
#define PARID_ACT_MOTORUNDERSHOOT		0x7C
#define PARID_ACT_MOTOROVERSHOOT		0x7D
#define PARID_ACT_NOMCUROVERSHOOT		0x7E
#define PARID_ACT_HMAXCUROVERSHOOT	0x7F

#define PARID_ACT_KP_PWMLIM					0x80
#define PARID_ACT_CURRENTLIMIT			0x81
#define PARID_ACT_MAXPWMOUTPUT			0x82

#endif
