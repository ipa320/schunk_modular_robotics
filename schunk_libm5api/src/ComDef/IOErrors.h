
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


#ifndef IOERRORS_H
#define IOERRORS_H

	// IO Device Types

#define TYPEID_IO_UNDEFINED				0		
#define TYPEID_IO_MAGELLAN				1		
#define TYPEID_IO_SPACEMASTER			2		
#define TYPEID_IO_METROLOGIC			3
#define TYPEID_IO_MODULEIO				4
#define TYPEID_IO_EMSMODULE				5
#define TYPEID_IO_DLRFTS				6
#define TYPEID_IO_SICK0D50				7
#define TYPEID_IO_COMPUTERBOARDS		8
#define TYPEID_IO_NI6527				9
#define TYPEID_IO_ME96					10
#define TYPEID_IO_MP55IO				11
#define TYPEID_IO_SCHUNKFTC				12
#define TYPEID_IO_DIRECTMOUSE			13
#define TYPEID_IO_DIRECTJOYSTICK		14
#define TYPEID_IO_DIRECTKEYBOARD		15

	// IO Device EMS Types

#define TYPEID_EMS_DIG24V_OUT			10
#define TYPEID_EMS_DIG24V_IN			11
#define TYPEID_EMS_ANA10V_OUT			12
#define TYPEID_EMS_ANA10V_IN			13
#define TYPEID_EMS_ANA25MA_OUT			14
#define TYPEID_EMS_ANA25MA_IN			15

	// IO Device Channel Types

#define CHANNELTYPEID_IO_READ			20		// undefined type 
#define CHANNELTYPEID_IO_BOOLREAD		21		
#define CHANNELTYPEID_IO_CHARREAD		22		
#define CHANNELTYPEID_IO_INTREAD		23		
#define CHANNELTYPEID_IO_FLOATREAD		24		
#define CHANNELTYPEID_IO_STRINGREAD		25		

#define CHANNELTYPEID_IO_WRITE			30		// undefined type for checking read/write
#define CHANNELTYPEID_IO_BOOLWRITE		31		
#define CHANNELTYPEID_IO_CHARWRITE		32		
#define CHANNELTYPEID_IO_INTWRITE		33		
#define CHANNELTYPEID_IO_FLOATWRITE		34		
#define CHANNELTYPEID_IO_STRINGWRITE	35		

	// IO Device Errors
	
#define	ERRID_IO						(-400L)				// io error
#define ERRID_IO_FUNCTIONNOTAVAILABLE  	(ERRID_IO - 1)	
#define ERRID_IO_NOINITSTRING			(ERRID_IO - 2)
#define	ERRID_IO_NOIONAME				(ERRID_IO - 3)
#define ERRID_IO_BADINITSTRING			(ERRID_IO - 4)
#define ERRID_IO_INITERROR				(ERRID_IO - 5)
#define ERRID_IO_ISINITIALIZED			(ERRID_IO - 6)
#define ERRID_IO_NOTINITIALIZED			(ERRID_IO - 7)
#define ERRID_IO_WRONGIOID				(ERRID_IO - 8)
#define ERRID_IO_WRONGCHANNELTYPEID		(ERRID_IO - 9)
#define ERRID_IO_WRONGCHANNELID			(ERRID_IO - 10)
#define ERRID_IO_WRONGCHANNELTYPE		(ERRID_IO - 11)
#define ERRID_IO_WRITEERROR				(ERRID_IO - 12)
#define ERRID_IO_READERROR				(ERRID_IO - 13)
#define ERRID_IO_WRONGCREATETHREAD		(ERRID_IO - 14)
#define ERRID_IO_NOLIBRARY				(ERRID_IO - 15)
#define ERRID_IO_CONNECTIONNOTINIT		(ERRID_IO - 16)
#define ERRID_IO_PARSERERROR			(ERRID_IO - 17)

#endif
