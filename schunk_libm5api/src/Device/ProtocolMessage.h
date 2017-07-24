/*
 * Copyright (c) 2012 SCHUNK GmbH & Co. KG
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
