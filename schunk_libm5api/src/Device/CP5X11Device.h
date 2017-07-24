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

#ifndef CCP5X11DEVICE_H
#define CCP5X11DEVICE_H

#include "../Device/ProtocolDevice.h"
#include "../Util/StopWatch.h"

class CCP5X11Device : public CProtocolDevice 
{
	private:

		// ---- private data ---------------------------------------------------- ;
	
		// ---- private auxiliary functions ------------------------------------- ;

	protected:

		// ---- protected data -------------------------------------------------- ;

			unsigned char m_hDevice; 
			unsigned char m_access;

			unsigned long m_uiTimeOut;
			unsigned char m_teaser;  
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
			CCP5X11Device();
			/// copy constructor
			CCP5X11Device(const CCP5X11Device& rclCP5X11Device);
			/// destructor
			virtual ~CCP5X11Device();

		// ---- operators ------------------------------------------------------- ;
		
			// assignment operator
			CCP5X11Device& operator=(const CCP5X11Device& rclCP5X11Device);

		// ---- query functions ------------------------------------------------- ;

		// ---- modify functions ------------------------------------------------ ;

			void setTimeOut(unsigned long uiTimeOut);

		// ---- I/O functions --------------------------------------------------- ;


		// ---- exec functions -------------------------------------------------- ;

			int init();
			int init(const char* acInitString);
			int exit();
};

#endif
