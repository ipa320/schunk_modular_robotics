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

#ifndef CPCANDEVICE_H
#define CPCANDEVICE_H

#include "../Device/ProtocolDevice.h"
#include <libpcan/libpcan.h>

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
