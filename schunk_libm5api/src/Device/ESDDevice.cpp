
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


#include "ESDDevice.h"
#if defined (_WIN32)
#include "../include/ntcan.h"
#endif
#if defined(__LINUX__)
#include "ntcan.h"
#endif
#if defined (__QNX__)
#include "../include/ntcan_qnx.h"
#endif

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

int CESDDevice::getDeviceError(int iErrorState)
{
	if(iErrorState == NTCAN_CONTR_BUSY)
	{
		warning("NTCAN_CONTR_BUSY");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_CONTR_OFF_BUS)
	{
		warning("NTCAN_CONTR_OFF_BUS");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_CONTR_WARN)
	{
		warning("NTCAN_CONTR_WARN");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_ID_NOT_ENABLED)
	{
		warning("NTCAN_ID_NOT_ENABLED");
		return ERRID_DEV_READERROR;
	}
#if defined(_WIN32)	
	else if(iErrorState == NTCAN_INVALID_HANDLE)
	{
		warning("NTCAN_INVALID_HANDLE");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_INVALID_HARDWARE)
	{
		warning("NTCAN_INVALID_HARDWARE");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_INVALID_PARAMETER)
	{
		warning("NTCAN_INVALID_PARAMETER");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_NET_NOT_FOUND)
	{
		warning("NTCAN_NET_NOT_FOUND");
		return ERRID_DEV_READERROR;
	}
#endif
	else if(iErrorState == NTCAN_MESSAGE_LOST)
	{
		warning("NTCAN_MESSAGE_LOST");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_NO_ID_ENABLED)
	{
		warning("NTCAN_NO_ID_ENABLED");
		return ERRID_DEV_READERROR;
	}
	else if(iErrorState == NTCAN_RX_TIMEOUT)
	{
		warning("NTCAN_RX_TIMEOUT");
		return ERRID_DEV_READTIMEOUT;
	}
	else if(iErrorState == NTCAN_TX_TIMEOUT)
	{
		warning("NTCAN_TX_TIMEOUT");
		return ERRID_DEV_WRITETIMEOUT;
	}
	else if(iErrorState == NTCAN_TX_ERROR)
	{
		warning("NTCAN_TX_ERROR");
		return ERRID_DEV_WRITEERROR;
	}
	return ERRID_DEV_WRITEERROR;
}

int CESDDevice::setBaudRate()
{
	int iRetVal = 0;
	m_iErrorState = 0;
	switch( m_iBaudRate )
	{
		case 125:
			m_uiBaudRate=0x06;	// 125k
			break;
		case 250:
			m_uiBaudRate=0x04;	// 250k
			break;
		case 500:
			m_uiBaudRate=0x02;	// 500k
			break;
		case 1000:
			m_uiBaudRate=0x00;	// 1000k
			break;
		default:
			m_uiBaudRate=0x04;	// 250k
			break;
	}

	iRetVal = canSetBaudrate(m_hDevice, m_uiBaudRate);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can set baudrate 0x%x failed Errorcode: %d", m_uiBaudRate, iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}	
	iRetVal = canSetBaudrate(m_hSyncDevice, m_uiBaudRate);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can set baudrate 0x%x failed Errorcode: %d", m_uiBaudRate, iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}	
	return m_iErrorState;
}

int CESDDevice::setMessageId(unsigned long uiMessageId)
{
	int iRetVal = 0;
	m_iErrorState = 0;
	iRetVal = canIdAdd(m_hDevice, uiMessageId);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can add ID failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}
	return m_iErrorState;
}

int CESDDevice::clearReadQueue()
{
	int iRetVal = 0;
	int32_t iNumberOfMessages = 1;
	static CMSG clESDProtocolMessage;
	m_iErrorState = 0;
	do
	{	
		iRetVal = canRead(m_hDevice, &clESDProtocolMessage, &iNumberOfMessages, NULL);			
	}while( ! ( ( iNumberOfMessages == 0 ) && ( iRetVal == NTCAN_SUCCESS ) ) );
	
	return m_iErrorState;
}

int CESDDevice::reinit(unsigned char ucBaudRateId)
{
	int i, iRetVal = 0;
	m_iErrorState = 0;
	if(!m_bInitFlag)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	switch(ucBaudRateId)
	{
		case BAUDRATEID_MOD_CAN_125K:
			m_iBaudRate = 125;
			break;
		case BAUDRATEID_MOD_CAN_250K:
			m_iBaudRate = 250;
			break;
		case BAUDRATEID_MOD_CAN_500K:
			m_iBaudRate = 500;
			break;
		case BAUDRATEID_MOD_CAN_1000K:
			m_iBaudRate = 1000;
			break;
	}
	iRetVal = canClose(m_hDevice);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can close failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_EXITERROR;
	}
	iRetVal = canClose(m_hSyncDevice);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can close failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_EXITERROR;
	}
	m_bInitFlag = false;
	iRetVal = canOpen(	
				m_iDeviceId,			// Net
				0,						// Mode
				m_uiQueueSize,			// TX Queue
				m_uiQueueSize,			// RX Queue
				20*m_uiTimeOut,			// Tx Timeout
				m_uiTimeOut,			// Rx Timeout
				&m_hDevice);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can open failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}
	/*iRetVal = canOpen(	
				m_iDeviceId,			// Net
				0,						// Mode
				1,						// TX Queue
				1,						// RX Queue
				600,					// Tx Timeout
				100,					// Rx Timeout
				&m_hSyncDevice);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can open failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}*/

	m_iErrorState = setBaudRate();
	if(m_iErrorState != 0)
		return m_iErrorState;

	for(i = 0; i <= m_iModuleCountMax; i++)
	{
		iRetVal = canIdAdd(m_hDevice, (MSGID_ACK + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
		iRetVal = canIdAdd(m_hDevice, (MSGID_STATE + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
	}

	for(i = 0; i < MAX_MP55; i++ )
	{
		iRetVal = canIdAdd(m_hDevice, (MSGID_MP55_RECV + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}

		iRetVal = canIdAdd(m_hDevice, (0x180 + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
	}

	for(i = 0; i < MAX_SCHUNK; i++ )
	{
		iRetVal = canIdAdd(m_hDevice, (MSGID_SCHUNK_RECV + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
	}

	iRetVal = canIdAdd(m_hSyncDevice, MSGID_ALL);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can add ID failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	m_iErrorState = clearReadQueue();
	if(m_iErrorState != 0)
		return m_iErrorState;

	if(m_iErrorState == 0)
		m_bInitFlag = true;

	updateModuleIdMap();
	return m_iErrorState;
}

int CESDDevice::readDevice(CProtocolMessage& rclProtocolMessage)
{
	int iRetVal = 0;
	int32_t iNumberOfMessages = 1;
	CMSG clESDProtocolMessage;
	m_iErrorState = 0;

#if defined ( _WIN32 )
	float fTimeDiff = 0;
	double dFrequency = 0;
	LARGE_INTEGER liTime, liTimeStart, liTimeEnd;

	QueryPerformanceFrequency( &liTime );
	dFrequency = liTime.LowPart;

	QueryPerformanceCounter( &liTimeStart );
	do
	{
		iNumberOfMessages = 1;
		iRetVal = canTake( m_hDevice, &clESDProtocolMessage, &iNumberOfMessages );	
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can read failed Errorcode: %d", iRetVal);
			m_iErrorState = getDeviceError(iRetVal);
			return m_iErrorState;
		}
		QueryPerformanceCounter( &liTimeEnd );
		fTimeDiff = (float)(liTimeEnd.LowPart - liTimeStart.LowPart) / (float)dFrequency * 1000;

		if( fTimeDiff >= m_uiTimeOut )
		{
			m_iErrorState = ERRID_DEV_READTIMEOUT;
			return m_iErrorState;
		}

	} while( iNumberOfMessages == 0 );
#else
	iRetVal = canRead(m_hDevice, &clESDProtocolMessage, &iNumberOfMessages, NULL);			
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can read failed Errorcode: %d", iRetVal);
		m_iErrorState = getDeviceError(iRetVal);
		return m_iErrorState;
	}
#endif

	rclProtocolMessage.m_uiMessageId = clESDProtocolMessage.id;
	rclProtocolMessage.m_ucMessageLength = clESDProtocolMessage.len;
	memcpy(rclProtocolMessage.m_aucMessageData, clESDProtocolMessage.data, rclProtocolMessage.m_ucMessageLength);
        printMessage(rclProtocolMessage,READ);

	return m_iErrorState;
}

int CESDDevice::writeDevice(CProtocolMessage& rclProtocolMessage)
{
	int iRetVal = 0;
	int32_t iNumberOfMessages = 1;
	CMSG clESDProtocolMessage;
	m_iErrorState = 0;

        //debug output

        printMessage(rclProtocolMessage,WRITE);
	clESDProtocolMessage.id = rclProtocolMessage.m_uiMessageId;
	clESDProtocolMessage.len = rclProtocolMessage.m_ucMessageLength;
	if(rclProtocolMessage.m_bRTRFlag)
		clESDProtocolMessage.len |= 0x10;
	memcpy(clESDProtocolMessage.data, rclProtocolMessage.m_aucMessageData, rclProtocolMessage.m_ucMessageLength);
	iRetVal = canWrite(m_hDevice, &clESDProtocolMessage, &iNumberOfMessages, NULL);
//	iRetVal = canSend(m_hDevice, &clESDProtocolMessage, &iNumberOfMessages);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can send failed Errorcode: %d", iRetVal);
		m_iErrorState = getDeviceError(iRetVal);
		return m_iErrorState;
	}

	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CESDDevice::CESDDevice() : m_hDevice(0), m_hSyncDevice(0), m_iDeviceId(-1), m_uiBaudRate(0), m_uiQueueSize(128), m_uiTimeOut(3)		// ESD C331
{
	initMessage("CESDDevice", g_iDebugLevel, g_bDebug, g_bDebugFile);
}

CESDDevice::CESDDevice(const CESDDevice& rclESDDevice)
{
	error(-1, "Sorry constructor is not implemented");
}

CESDDevice::~CESDDevice()
{
	exit();
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CESDDevice& CESDDevice::operator=(const CESDDevice& rclESDDevice)
{
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

void CESDDevice::setQueueSize(unsigned short uiQueueSize)
{
	m_uiQueueSize = uiQueueSize;
}

void CESDDevice::setTimeOut(unsigned long uiTimeOut)
{
	m_uiTimeOut= uiTimeOut;
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

int CESDDevice::init()
{
	return init(m_acInitString);
}

int CESDDevice::init(const char* acInitString)
{
	InitializeCriticalSection(&m_csDevice);
	int i, iRetVal = 0;
	int txTimeOut = 0;
	char* pcToken;
	char acString[128];
	if(m_bInitFlag)
	{
		warning("device already initialized");
		m_iErrorState = ERRID_DEV_ISINITIALIZED;
		return m_iErrorState;
	}
	m_iDeviceId = -1;
	m_iErrorState = 0;
	strncpy(m_acInitString,acInitString,128);
	strncpy(acString,acInitString,128);
	pcToken = strtok( acString, ":" );
	if( !pcToken )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	if( strcmp( pcToken, "ESD" ) != 0 )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	pcToken = strtok( NULL, "," );
	if( !pcToken )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	m_iDeviceId = atoi(pcToken);

	pcToken = strtok( NULL, "," );
	if( !pcToken )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	m_iBaudRate = atoi(pcToken);

#if defined(__LINUX__)
	m_uiTimeOut = 6;
#endif
#if defined (_WIN32)
	switch( m_iBaudRate )
	{
	case 125:
	case 250:
		m_uiTimeOut = 4;
		break;
	case 500:
		m_uiTimeOut = 3;
		break;
	case 1000:
		m_uiTimeOut = 2;
		break;
	default:
		m_uiTimeOut = 10;
		break;
	}
#endif

	try
	{
		iRetVal = canOpen(	
					m_iDeviceId,			// Net
					0,						// Mode
					m_uiQueueSize,			// TX Queue
					m_uiQueueSize,			// RX Queue
					m_uiTimeOut,			// Tx Timeout
					m_uiTimeOut,			// Rx Timeout
					&m_hDevice);
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can open failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
		iRetVal = canOpen(	
					m_iDeviceId,			// Net
					0,						// Mode
					1,						// TX Queue
					1,						// RX Queue
					600,					// Tx Timeout
					100,					// Rx Timeout
					&m_hSyncDevice);
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can open failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
	}
	catch(...)
	{
			warning("init ESD device failed no library found");
			m_iErrorState = ERRID_DEV_NOLIBRARY;
			return m_iErrorState;
	}

	for(i = 0; i <= m_iModuleCountMax; i++)
	{
		iRetVal = canIdAdd(m_hDevice, (MSGID_ACK + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
		iRetVal = canIdAdd(m_hDevice, (MSGID_STATE + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
	}

	for(i = 0; i <= MAX_MP55; i++ )
	{
		iRetVal = canIdAdd(m_hDevice, (MSGID_MP55_RECV + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}

		iRetVal = canIdAdd(m_hDevice, (0x180 + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
		iRetVal = canIdAdd(m_hDevice, (0x600 + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
	}

	for(i = 0; i < MAX_SCHUNK; i++ )
	{
		iRetVal = canIdAdd(m_hDevice, (MSGID_SCHUNK_RECV + i));
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can add ID failed Errorcode: %d", iRetVal);
			getDeviceError(iRetVal);
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
	}
	
	iRetVal = canIdAdd(m_hSyncDevice, MSGID_ALL);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can add ID failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	m_iErrorState = setBaudRate();
	if(m_iErrorState != 0)
		return m_iErrorState;

	m_iErrorState = clearReadQueue();
	if(m_iErrorState != 0)
		return m_iErrorState;

	if(m_iErrorState == 0)
		m_bInitFlag = true;

	updateModuleIdMap();
	return m_iErrorState;
}

int CESDDevice::exit()
{
	int iRetVal = 0;
	m_iErrorState = 0;
	if(!m_bInitFlag)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	EnterCriticalSection(&m_csDevice);
	iRetVal = canClose(m_hDevice);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can close failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_EXITERROR;
	}
	iRetVal = canClose(m_hSyncDevice);
	if(iRetVal != NTCAN_SUCCESS)
	{
		warning("can close failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_EXITERROR;
	}
	m_bInitFlag = false;
	LeaveCriticalSection(&m_csDevice);
	DeleteCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CESDDevice::waitForStartMotionAll()
{
	int iRetVal = 0;
	bool bRecieved = false;
	int32_t iNumberOfMessages = 1;
	CMSG clESDProtocolMessage;
	m_iErrorState = 0;

	do
	{	
		iRetVal = canRead(m_hSyncDevice, &clESDProtocolMessage, &iNumberOfMessages, NULL);			
		if(iRetVal != NTCAN_SUCCESS)
		{
			warning("can read failed Errorcode: %d", iRetVal);
			m_iErrorState = getDeviceError(iRetVal);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clESDProtocolMessage.id != MSGID_ALL)
		{
			debug(1, "received CAN-ID %x, expected %x", clESDProtocolMessage.id, MSGID_ALL);
			bRecieved = false;
		}
		if(clESDProtocolMessage.data[0] != CMDID_STARTMOVE)
		{
			debug(1, "wrong command ID");
			bRecieved = false;
		}
	}
	while(!bRecieved);
	return m_iErrorState;
}

