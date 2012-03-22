
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


#ifndef CPCANDEVICE_H
#define CPCANDEVICE_H

#include "../Device/ProtocolDevice.h"
#include "libpcan.h"

class CPCanDevice : public CProtocolDevice  
{
	private:

		// ---- private data ---------------------------------------------------- ;
	
		// ---- private auxiliary functions ------------------------------------- ;

	protected:

		// ---- protected data -------------------------------------------------- ;

			HANDLE m_handle;
			int m_hDevice;
			int m_hSyncDevice;
			int m_iDeviceId;
			bool m_bInitialized;
			unsigned long  m_uiBaudRate;	
			unsigned short m_uiQueueSize;
			unsigned long m_uiTimeOut;
                        int m_iNoOfRetries;

		// ---- protected auxiliary functions ----------------------------------- ;

			int getDeviceError(int iErrorState);
			int setBaudRate();
			int setBaudRate(unsigned char iBaudRate);
			int setMessageId(unsigned long uiMessageId);
			int clearReadQueue();
			int reinit(unsigned char ucBaudRateId);
			int readDevice(CProtocolMessage& rclProtocolMessage);
			int writeDevice(CProtocolMessage& rclProtocolMessage);
			char * m_DeviceName;

	public:

		// ---- public data ----------------------------------------------------- ;


	
		// ---- constructors / destructor --------------------------------------- ;

			/// default constructor
			CPCanDevice();
			/// copy constructor
			CPCanDevice(const CPCanDevice& rclPCanDevice);
			/// destructor
			virtual ~CPCanDevice();

		// ---- operators ------------------------------------------------------- ;
		
			// assignment operator
			CPCanDevice& operator=(const CPCanDevice& rclPCanDevice);

		// ---- query functions ------------------------------------------------- ;

		// ---- modify functions ------------------------------------------------ ;
			
			void setQueueSize(unsigned short uiQueueSize);
			void setTimeOut(unsigned long uiTimeOut);

		// ---- I/O functions --------------------------------------------------- ;

		// ---- exec functions -------------------------------------------------- ;

			int init();
			int init(unsigned long baudRate);
			int init(const char* acInitString);
			int exit();
			int waitForStartMotionAll();
};

#endif
