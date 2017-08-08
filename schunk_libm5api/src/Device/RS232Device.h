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

#ifndef CRS232DEVICE_H
#define CRS232DEVICE_H

// ---- local includes ------------------------------------------------------ ;

#include "../Device/ProtocolDevice.h"
#include "../Util/StopWatch.h"

// ---- global includes ----------------------------------------------------- ;

#ifdef __LINUX__
	#include <fcntl.h>
	#include <termios.h>	
#endif
#ifdef __QNX__
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <sys/select.h>
	#include <fcntl.h>
	#include <termios.h>	
#endif

class CRS232Device : public CProtocolDevice  
{
	private:

		// ---- private data ---------------------------------------------------- ;
	
		// ---- private auxiliary functions ------------------------------------- ;

	protected:

		// ---- protected data -------------------------------------------------- ;

#if defined (_WIN32)
			HANDLE m_hDevice;
#endif
#if defined(__LINUX__)
			int m_hDevice;
#endif
#if defined (__QNX__)
			int m_hDevice;
#endif
			int m_iDeviceId;
			unsigned long  m_uiBaudRate;	
			unsigned long m_uiTimeOut;
			CStopWatch m_clTimer;

		// ---- protected auxiliary functions ----------------------------------- ;

			int getDeviceError(int iErrorState);
			int setBaudRate();
			int setMessageId(unsigned long uiMessageId);
			int clearReadQueue();
			int reinit(unsigned char ucBaudRateId);
			int readDevice(CProtocolMessage& rclProtocolMessage);
			int writeDevice(CProtocolMessage& rclProtocolMessage);

	public:

		// ---- public data ----------------------------------------------------- ;


	
		// ---- constructors / destructor --------------------------------------- ;

			/// default constructor
			CRS232Device();
			/// copy constructor
			CRS232Device(const CRS232Device& rclRS232Device);
			/// destructor
			virtual ~CRS232Device();

		// ---- operators ------------------------------------------------------- ;
		
			// assignment operator
			CRS232Device& operator=(const CRS232Device& rclRS232Device);

		// ---- query functions ------------------------------------------------- ;

		// ---- modify functions ------------------------------------------------ ;
			
			void setQueueSize(unsigned short uiQueueSize);
			void setTimeOut(unsigned long uiTimeOut);

		// ---- I/O functions --------------------------------------------------- ;

		// ---- exec functions -------------------------------------------------- ;

			int init();
			int init(const char* acInitString);
			int exit();
};

#endif
