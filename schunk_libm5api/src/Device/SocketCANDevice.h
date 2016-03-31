/*****************************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group, 
 * Institute for Anthropomatics and Robotics (IAR) - 
 * Intelligent Process Control and Robotics (IPR), 
 * Karlsruhe Institute of Technology (KIT)
 *
 * Author: Tulbure Andreea, email: andreea_tulbure@yahoo.de
 *         Denis Stogl, email: denis.stogl@kit.edu
 *
 * Date of creation: 03.2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided
 * that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************/

#ifndef SocketCANDevice_INCLUDEDEF_H
#define SocketCANDevice_INCLUDEDEF_H
//-----------------------------------------------

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <cerrno>
#include <cstring>

#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <string.h>

#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif


//-----------------------------------------------

#include <iostream>
#include <cstdio>
#include <libpcan/libpcan.h>
#include "ProtocolDevice.h"

//-----------------------------------------------

class SocketCANDevice: public CProtocolDevice {

private:

	// ---- private data ---------------------------------------------------- ;

	// ---- private auxiliary functions ------------------------------------- ;

protected:
	// ---- protected data ---------------------------------------------------- ;
	bool m_bInitialized;
	int m_iDeviceId;
	int m_iNoOfRetries;
	unsigned short m_uiQueueSize;
	unsigned long m_uiTimeOut;
	char * m_DeviceName;


	// ---- protected auxiliary functions ------------------------------------- ;
	int getDeviceError(int iErrorState);
	int setBaudRate();
	int setBaudRate(unsigned char iBaudRate);
	int setMessageId(unsigned long uiMessageId);
	int clearReadQueue();
	int reinit(unsigned char ucBaudRateId);
	int readDevice(CProtocolMessage& rclProtocolMessage);
	int writeDevice(CProtocolMessage& rclProtocolMessage);

public:

	// ---- public data ----------------------------------------------------- ;

	// ---- constructors / destructor --------------------------------------- ;

	// default constructor
	SocketCANDevice();
	SocketCANDevice(const SocketCANDevice& rclSocketCANDevice);
	~SocketCANDevice();
	
	// ---- operators ------------------------------------------------------- ;
	// assignment operator
	SocketCANDevice& operator=(const SocketCANDevice& rclSocketCANDevice);
	
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
