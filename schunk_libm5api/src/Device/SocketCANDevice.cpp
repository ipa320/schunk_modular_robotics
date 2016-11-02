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

#include "SocketCANDevice.h"
#include <errno.h>

struct can_frame frame;
// ========================================================================== ;
//                                                                            ;
// ---- private auxiliary functions ----------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

// ========================================================================== ;
//                                                                            ;
// ---- protected auxiliary functions --------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

//possible incomplete
//
int SocketCANDevice::getDeviceError(int iErrorState) {
	int error = ERRID_DEV_WRITEERROR;

	if (((unsigned int) iErrorState & 0xFFFF) & CAN_ERR_QRCVEMPTY) {
		warning("receive queue is empty");
		error = ERRID_DEV_WRITEERROR;
	}
	if (iErrorState & CAN_ERR_OVERRUN) {
		warning("receive buffer overrun");
		error = ERRID_DEV_READERROR;
	}
	if (iErrorState & CAN_ERR_XMTFULL) {
		warning("transmit buffer full");
		error = ERRID_DEV_WRITEERROR;
	}
	if (iErrorState & CAN_ERR_BUSOFF) {
		warning("CAN_ERR_OFF_BUS");
		error = ERRID_DEV_READERROR;
	}
	if (iErrorState & CAN_ERR_ILLPARAMTYPE) {
		warning("CAN_ERR_ILLPARAMTYPE");
		error = ERRID_DEV_READERROR;
	}
	if (((unsigned int) iErrorState & 0xFFFF) & CAN_ERR_QXMTFULL) {
		warning("transmit queue full");
		error = ERRID_DEV_WRITEERROR;
	}
	if (iErrorState & CAN_ERR_BUSLIGHT) {
		warning("bus error");
		error = ERRID_DEV_WRITEERROR;
	}
	if (((unsigned int) iErrorState & 0xFFFF) & CAN_ERR_BUSHEAVY) {
		warning("bus error");
		error = ERRID_DEV_WRITEERROR;
	}
	if (((unsigned int) iErrorState & 0xFFFF) & CAN_ERR_RESOURCE) {
		warning("can't create resource");
		error = ERRID_DEV_WRITEERROR;
	}

	return error;
}

int SocketCANDevice::setBaudRate(unsigned char iBaudRate) {
	return setBaudRate();
}

int SocketCANDevice::setBaudRate() {
	return m_iErrorState;
}

int SocketCANDevice::setMessageId(unsigned long uiMessageId) {
	return m_iErrorState;
}

int SocketCANDevice::clearReadQueue() {

	int iRetVal = 0;
	can_frame frame;

	debug(1, "entering SocketCANDevice::clearReadQueue()...\n");
	m_iErrorState = 0;
	do {
		debug(1, "Trying to read messages ...");
		iRetVal = read(m_iDeviceId, &frame, sizeof(frame));
		debug(0, " 0x%04x\n", iRetVal);

	} while (iRetVal != CAN_ERR_QRCVEMPTY);
	return iRetVal;
}

int SocketCANDevice::reinit(unsigned char ucBaudRateId) {

	return m_iErrorState;
}

int SocketCANDevice::readDevice(CProtocolMessage& rclProtocolMessage) {
// set frame to zero
	frame.data[0] = 0;
	frame.data[1] = 0;
	frame.data[2] = 0;
	frame.data[3] = 0;
	frame.data[4] = 0;
	frame.data[5] = 0;
	frame.data[6] = 0;
	frame.data[7] = 0;
	frame.can_dlc = 8; //rclProtocolMessage.m_ucMessageLength;
	frame.can_id = 0; //rclProtocolMessage.m_uiMessageId;
	int iCount = 0;
	int bRet = 1, iRet = 0;
	int nrRead = 0;

	debug(1, "Trying to read Device");

	iRet = read(m_iDeviceId, &frame, sizeof(frame));

	if (iRet >= 0) // no problem occured
			{

		debug(1, "sizeof(frame)", sizeof(frame));
		debug(1, "bytes read: %d", iRet);
		debug(1, "id: %d", frame.can_id);
		debug(1, "data: %c ", frame.data);
		debug(1, "length: %d ", frame.can_dlc);

// copy read data from frame to ProtocolMessage (byte-wise or string)
		rclProtocolMessage.m_uiMessageId = frame.can_id;
		rclProtocolMessage.m_ucMessageLength = frame.can_dlc;
		memcpy(rclProtocolMessage.m_aucMessageData, frame.data,
				rclProtocolMessage.m_ucMessageLength);
		bRet = 0;
	} else // reading problem occured
	{
		// no message, errror
		if (iCount > 26)
			printf("error in SocketCANDevice::readDevice()");
		iCount++;
		bRet = 1;
	}

	debug(1, "bRet before return from SocketCANDevice::readDevice: %d ", bRet);
	return bRet;

}

int SocketCANDevice::writeDevice(CProtocolMessage& rclProtocolMessage) {

	debug(1, "SocketCANDevice::writeDevice ");

	int bytes_sent = -1;

	debug(1, "m_iDeviceId %d", m_iDeviceId);

	if (m_bInitialized == false)
		return false;

	frame.can_dlc = (int) rclProtocolMessage.m_ucMessageLength;

	debug(1, "frame can_dlc: %d", frame.can_dlc);
	debug(1, "clProtocolMessage.m_ucMessageLength: %d",
			(unsigned int) rclProtocolMessage.m_ucMessageLength);

	frame.can_id = rclProtocolMessage.m_uiMessageId;

	debug(1, "frame can_id_%d ", frame.can_id);
	debug(1, "rclProtocolMessage.m_uiMessageId: %d ",
			rclProtocolMessage.m_uiMessageId);

// copy data from ProtocolMessage to frame.data (byte-wise or string)
	for (int i = 0; i < (unsigned int) rclProtocolMessage.m_ucMessageLength;
			i++) {

		frame.data[i] = rclProtocolMessage.m_aucMessageData[i];
		debug(1, "rclProtocolMessage.m_aucMessageData: %c ",
				rclProtocolMessage.m_aucMessageData);
		debug(1, "frame data[%d]: %c", i, frame.data);
	}
	debug(1, "sizeof frame : %d", sizeof(frame));

	int bRet = 1;

//writing
	bytes_sent = write(m_iDeviceId, &frame, sizeof(frame));
	usleep(10000);
	if (bytes_sent < 0) {
		debug(1, "error in SocketCANDevice::writeDevice: ");
		bRet = 1;
	}

	if (bytes_sent > 0) {
		bRet = 0;
		debug(1, "bytes sent in SocketCANDevice::writeDevice: %d ", bytes_sent);
	}

	return bRet;

}

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

SocketCANDevice::SocketCANDevice() {
	m_bInitialized = false;
}

SocketCANDevice::SocketCANDevice(const SocketCANDevice& rclSocketCANDevice) {
	error(-1, "Sorry constructor is not implemented");
}

SocketCANDevice::~SocketCANDevice() {
	if (m_bInitialized) {
		this->exit();
	}
}
// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

SocketCANDevice& SocketCANDevice::operator=(const SocketCANDevice& rclSocketCANDevice) {
	error(-1, "Sorry operator= is not implemented");
	return *this;
}

// ========================================================================== ;
//                                                                            ;
// ---- query functions ----------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

// ========================================================================== ;
//                                                                            ;
// ---- modify functions ---------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

void SocketCANDevice::setQueueSize(unsigned short uiQueueSize) {
	m_uiQueueSize = uiQueueSize;
}

void SocketCANDevice::setTimeOut(unsigned long uiTimeOut) {
	m_uiTimeOut = uiTimeOut;
}

// ========================================================================== ;
//                                                                            ;
// ---- I/O functions ------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

// ========================================================================== ;
//                                                                            ;
// ---- exec functions ------------------------------------------------------ ;
//                                                                            ;
// ========================================================================== ;

int SocketCANDevice::init() {
	return init(0x014);
}

int SocketCANDevice::init(unsigned long baudRate) {
	return m_iErrorState;
}

int SocketCANDevice::init(const char* acInitString) {

	printf("Trying to open CAN on can0 ...");
	m_iErrorState = 0;
	setTimeOut(100000);
	m_iDeviceId = socket(PF_CAN, SOCK_RAW, CAN_RAW);

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = m_uiTimeOut;

	if (setsockopt(m_iDeviceId, SOL_SOCKET, SO_RCVTIMEO, (char *) &timeout,
			sizeof(struct timeval)) < 0)
		error("setsockopt failed\n");

	struct ifreq ifr;
	char* pcToken;
	char acString[128];
	strncpy(m_acInitString, acInitString, 128);
	strncpy(acString, acInitString, 128);
	pcToken = strtok(acString, ",");
	std::string pcTokenStrInit(pcToken);
	pcToken = strtok(pcToken, ":");
	if (!pcToken) {
		m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	if (strcmp(pcToken, "SOCKETCAN") != 0) {
		m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	std::string pcTokenStr(pcToken);
	std::string devName = pcTokenStrInit.substr(pcTokenStr.length() + 1, 4);
	strcpy(ifr.ifr_name, devName.c_str());
	m_DeviceName = ifr.ifr_name;
	debug(1,"name: %x",*m_DeviceName);
	int ret = ioctl(m_iDeviceId, SIOCGIFINDEX, &ifr);
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret1 = bind(m_iDeviceId, (struct sockaddr*) &addr, sizeof(addr));

	m_bInitFlag = true;

	if (!m_iDeviceId) {
		printf("Cannot open CAN on USB:\n");
	} else {
		printf("Open CAN on USB suceeded!\n");
		m_bInitialized = true;
	}

	updateModuleIdMap();

	debug(1, "finished updateModuleIdMap");
	debug(1,
		 "m_iErrorState before returning of SocketCANDevice::init(const char* acInitString): %d",
			m_iErrorState);

	return m_iErrorState;
}

int SocketCANDevice::exit() {
	int iRetVal = 0;
	if (!m_bInitialized) {
		warning("exit:device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	EnterCriticalSection(&m_csDevice);
	iRetVal = CAN_ERR_OK;
	if (iRetVal != CAN_ERR_OK) {
		warning("can close failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_EXITERROR;
	}
	m_bInitFlag = false;
	LeaveCriticalSection(&m_csDevice);
	DeleteCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int SocketCANDevice::waitForStartMotionAll() {
	int iRetVal = 0;
	TPCANRdMsg TPCMsg;
	TPCMsg.Msg.LEN = 8;
	TPCMsg.Msg.MSGTYPE = 0;
	TPCMsg.Msg.ID = 0;
	bool bRecieved = false;
	can_frame frame;
	m_iErrorState = 0;
	iRetVal = read(m_iDeviceId, &frame, sizeof(frame));
	m_iErrorState = 0;

	do {
		iRetVal = read(m_iDeviceId, &frame, sizeof(frame));
		if (iRetVal != CAN_ERR_OK) {
			warning("can read failed Errorcode: 0x%04x", iRetVal);
			m_iErrorState = getDeviceError(iRetVal);
			return m_iErrorState;
		}
		bRecieved = true;
		if (frame.can_id != MSGID_ALL) {
			debug(1, "received CAN-ID %x, expected %x", TPCMsg.Msg.ID,
					MSGID_ALL);
			bRecieved = false;
		}
		if (frame.data[0] != CMDID_STARTMOVE) {
			debug(1, "wrong command ID");
			bRecieved = false;
		}
	} while (!bRecieved);
	return m_iErrorState;
}

