
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




#ifndef DEVICEERRORS_H
#define DEVICEERRORS_H

	// Device Errors
	
#define	ERRID_DEV							(-200L)		// device message

#define ERRID_DEV_FUNCTIONNOTAVAILABLE  	(ERRID_DEV - 1)	
#define ERRID_DEV_NOINITSTRING				(ERRID_DEV - 2)
#define ERRID_DEV_NODEVICENAME				(ERRID_DEV - 3)
#define ERRID_DEV_BADINITSTRING				(ERRID_DEV - 4)
#define ERRID_DEV_INITERROR					(ERRID_DEV - 5)
#define ERRID_DEV_NOTINITIALIZED			(ERRID_DEV - 6)
#define ERRID_DEV_WRITEERROR				(ERRID_DEV - 7)
#define ERRID_DEV_READERROR					(ERRID_DEV - 8)
#define ERRID_DEV_WRITETIMEOUT				(ERRID_DEV - 9)
#define ERRID_DEV_READTIMEOUT				(ERRID_DEV - 10)
#define ERRID_DEV_WRONGMESSAGEID			(ERRID_DEV - 11)
#define ERRID_DEV_WRONGCOMMANDID			(ERRID_DEV - 12)
#define ERRID_DEV_WRONGPARAMETERID			(ERRID_DEV - 13)
#define ERRID_DEV_EXITERROR					(ERRID_DEV - 14)
#define ERRID_DEV_NOMODULES					(ERRID_DEV - 15)
#define ERRID_DEV_WRONGDEVICEID				(ERRID_DEV - 16)
#define ERRID_DEV_NOLIBRARY					(ERRID_DEV - 17)
#define ERRID_DEV_ISINITIALIZED				(ERRID_DEV - 18)
#define ERRID_DEV_WRONGEMSMODULEID			(ERRID_DEV - 19)
#define ERRID_DEV_EMSNOTINITIALIZED			(ERRID_DEV - 20)
#define ERRID_DEV_EMSMAXNUMBER				(ERRID_DEV - 21)
#define ERRID_DEV_EMSINITERROR				(ERRID_DEV - 22)
#define ERRID_DEV_WRONGEMSTYPE				(ERRID_DEV - 23)
#define ERRID_DEV_WRONGEMSCHANNELID			(ERRID_DEV - 24)
#define ERRID_DEV_WRONGMP55MODULEID			(ERRID_DEV - 25)
#define ERRID_DEV_WRONGSCHUNKMODULEID		(ERRID_DEV - 26)
#define ERRID_DEV_WRONGMODULEID				(ERRID_DEV - 27)
#define ERRID_DEV_MODULEERROR				(ERRID_DEV - 28)
#define ERRID_DEV_WAITTIMEOUT				(ERRID_DEV - 29)
#define ERRID_DEV_CONNECTIONNOTINIT			(ERRID_DEV - 30)
#define ERRID_DEV_PARSERERROR				(ERRID_DEV - 31)
#endif
