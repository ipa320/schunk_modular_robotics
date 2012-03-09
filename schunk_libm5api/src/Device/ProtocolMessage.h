
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


#ifndef CPROTOCOLMESSAGE_H
#define CROTOCOLMESSAGE_H

#include "../Util/GlobalDefines.h"

class CProtocolMessage  
{
	public:

		// ---- public data ----------------------------------------------------- ;

		unsigned long m_uiMessageId;		//  Identifier 11-/29-Bit
											//  11 bit in low word, 29 bit complete
		unsigned char m_ucMessageLength;	//  number of data bytes (0-8)
		unsigned char m_aucMessageData[8];	//  Array for up to 8 data bytes
		unsigned char m_ucMessageState;		//  Bit coded information for state
		bool m_bRTRFlag;					//  RTR-Bit: 0=Dataframe, 1=Remoteframe
		double m_fTime;						//  time stamp in sec
		int m_iModuleId;					//  module bus address
	
		// ---- constructors / destructor --------------------------------------- ;

			/// default constructor
			CProtocolMessage();
			/// copy constructor
			CProtocolMessage(const CProtocolMessage& rclProtocolMessage);
			/// destructor
			~CProtocolMessage();
		// ---- operators ------------------------------------------------------ ;
		
			// assignment operator
			CProtocolMessage& operator=(const CProtocolMessage& rclProtocolMessage);
};

typedef struct
{ 
	unsigned char m_aucMessageId[2];
	unsigned char m_aucMessageData[8];
	unsigned char m_ucMessageLength;
} CRS232Message;

typedef union
{
	unsigned char aucData[4];
	char acData[4];
	unsigned short auiData[2];
	short aiData[2];
	unsigned long uiData;
	long iData;
	float fData;
} CProtocolData;

#endif
