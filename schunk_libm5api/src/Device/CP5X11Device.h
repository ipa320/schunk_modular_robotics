
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
