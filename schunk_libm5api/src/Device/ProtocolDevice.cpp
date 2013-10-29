
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


#include "ProtocolDevice.h"

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

int CProtocolDevice::write8Bytes(int iModuleId, bool bAck, void* pBytes ) 
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	static CProtocolMessage clWrite, clRead;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	memcpy( clWrite.m_aucMessageData, pBytes, 8 );
	clWrite.m_ucMessageLength = 8;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

//	warning("Sending %03x  %02x %02x %02x %02x %02x %02x %02x %02x", clWrite.m_uiMessageId, 
//		clWrite.m_aucMessageData[0], clWrite.m_aucMessageData[1], clWrite.m_aucMessageData[2], clWrite.m_aucMessageData[3],
//		clWrite.m_aucMessageData[4], clWrite.m_aucMessageData[5], clWrite.m_aucMessageData[6], clWrite.m_aucMessageData[7] );

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	if( bAck )
	{	
			m_iErrorState = readDevice(clRead);
			if(m_iErrorState != 0)
			{
				//warning("wrong readDevice ErrorCode %i", m_iErrorState);
				LeaveCriticalSection(&m_csDevice);
				return m_iErrorState;
			}
			memcpy( pBytes, clRead.m_aucMessageData, clRead.m_ucMessageLength );
	}
	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::getUnsignedLong(int iModuleId, unsigned long* puiData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clRead;
	static CProtocolData clData;
	*puiData = 0;

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_STATE + iModuleId)
		{	
			debug(1, "getUnsignedLong: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_STATE + iModuleId );
			bRecieved = false;
		}
	}
	while(!bRecieved);
	clData.aucData[0] = clRead.m_aucMessageData[0];
	clData.aucData[1] = clRead.m_aucMessageData[1];
	clData.aucData[2] = clRead.m_aucMessageData[2];
	clData.aucData[3] = clRead.m_aucMessageData[3];
	*puiData = clData.uiData;
	
	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::readChar(int iModuleId, int iCommandId, int iParameterId, char* pcData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1, "readChar: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readChar: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readChar: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	*pcData = clRead.m_aucMessageData[2];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readUnsignedChar(int iModuleId, int iCommandId, int iParameterId, unsigned char* pucData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
	m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readUnsignedChar: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readUnsignedChar: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readUnsignedChar: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	*pucData = clRead.m_aucMessageData[2];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readShort(int iModuleId, int iCommandId, int iParameterId, short* piData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readShort: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readShort: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readShort: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	*piData = clData.aiData[0];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readUnsignedShort(int iModuleId, int iCommandId, int iParameterId, unsigned short* puiData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readUnsignedShort: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readUnsignedShort: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readUnsignedShort: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	*puiData = clData.auiData[0];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readLong(int iModuleId, int iCommandId, int iParameterId, long* piData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readLong: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readLong: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readLong: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*piData = clData.iData;
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readUnsignedLong(int iModuleId, int iCommandId, int iParameterId, unsigned long* puiData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	do
	{
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning( "wrong readDevice ErrorCode %i", m_iErrorState );
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readUnsignedLong: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}

		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readUnsignedLong: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}

		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readUnsignedLong: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*puiData = clData.uiData;
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readFloat(int iModuleId, int iCommandId, int iParameterId, float* pfData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	
	do
	{
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
//			warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readFloat: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readFloat: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readFloat: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*pfData = clData.fData;
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long* piData, unsigned char* pucData1, unsigned char* pucData2)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readLongUnsignedChars: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readLongUnsignedChars: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readLongUnsignedChars: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*piData = clData.iData;
	*pucData1 = clRead.m_aucMessageData[6];
	*pucData2 = clRead.m_aucMessageData[7];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::readFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float* pfData, unsigned char* pucData1, unsigned char* pucData2)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_GET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"readFloatUnsignedChars: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"readFloatUnsignedChars: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"readFloatUnsignedChars: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*pfData = clData.fData;
	*pucData1 = clRead.m_aucMessageData[6];
	*pucData2 = clRead.m_aucMessageData[7];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeChar(int iModuleId, int iCommandId, int iParameterId, char cData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = cData;
	clWrite.m_ucMessageLength = 3;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeChar: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeChar: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeChar: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeUnsignedChar(int iModuleId, int iCommandId, int iParameterId, unsigned char ucData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = ucData;
	clWrite.m_ucMessageLength = 3;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeUnsignedChar: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeUnsignedChar: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeUnsignedChar: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeShort(int iModuleId, int iCommandId, int iParameterId, short iData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.aiData[0] = iData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_ucMessageLength = 4;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeShort: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeShort: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeShort: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeUnsignedShort(int iModuleId, int iCommandId, int iParameterId, unsigned short uiData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.auiData[0] = uiData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_ucMessageLength = 4;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeUnsignedShort: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeUnsignedShort: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeUnsignedShort: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeLong(int iModuleId, int iCommandId, int iParameterId, long iData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.iData = iData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clWrite.m_ucMessageLength = 6;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeLong: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeLong: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeLong: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeUnsignedLong(int iModuleId, int iCommandId, int iParameterId, unsigned long uiData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.uiData = uiData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clWrite.m_ucMessageLength = 6;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeUnsignedLong: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeUnsignedLong: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeUnsignedLong: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeFloat(int iModuleId, int iCommandId, int iParameterId, float fData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.fData = fData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clWrite.m_ucMessageLength = 6;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeFloat: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeFloat: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeFloat: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeAll(int iCommandId, int iParameterId)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	static CProtocolMessage clWrite, clRead;

	clWrite.m_uiMessageId = MSGID_ALL;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_ucMessageLength = 2;
	clWrite.m_iModuleId = 0;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeCommand(int iModuleId, int iCommandId)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_ucMessageLength = 1;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeCommand: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeCommand: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeLongShort(int iModuleId, int iCommandId, int iParameterId, long iData, short iTime)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clData.iData = iData;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clData.aiData[0] = iTime;
	clWrite.m_aucMessageData[6] = clData.aucData[0];
	clWrite.m_aucMessageData[7] = clData.aucData[1];
	clWrite.m_ucMessageLength = 8;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeLongShort: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeLongShort: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeLongShort: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeFloatShort(int iModuleId, int iCommandId, int iParameterId, float fData, short iData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clData.fData = fData;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clData.aiData[0] = iData;
	clWrite.m_aucMessageData[6] = clData.aucData[0];
	clWrite.m_aucMessageData[7] = clData.aucData[1];
	clWrite.m_ucMessageLength = 8;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeFloatShort: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeFloatShort: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeFloatShort: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeShortReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, short iData, long* piData, unsigned char* pucData1, unsigned char* pucData2)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.aiData[0] = iData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_ucMessageLength = 4;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeShortReadUnsignedLongChars: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeShortReadUnsignedLongChars: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeShortReadUnsignedLongChars: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*piData = clData.iData;
	*pucData1 = clRead.m_aucMessageData[6];
	*pucData2 = clRead.m_aucMessageData[7];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeLongReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long iData, long* piData, unsigned char* pucData1, unsigned char* pucData2)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.iData = iData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clWrite.m_ucMessageLength = 6;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeLongReadUnsignedLongChars: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeLongReadUnsignedLongChars: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeLongReadUnsignedLongChars: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*piData = clData.iData;
	*pucData1 = clRead.m_aucMessageData[6];
	*pucData2 = clRead.m_aucMessageData[7];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeLongShortReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long iData1, short iData2, long* piData, unsigned char* pucData1, unsigned char* pucData2)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clData.iData = iData1;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clData.aiData[0] = iData2;
	clWrite.m_aucMessageData[6] = clData.aucData[0];
	clWrite.m_aucMessageData[7] = clData.aucData[1];
	clWrite.m_ucMessageLength = 8;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeLongShortReadUnsignedLongChars: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeLongShortReadUnsignedLongChars: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeLongShortReadUnsignedLongChars: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*piData = clData.iData;
	*pucData1 = clRead.m_aucMessageData[6];
	*pucData2 = clRead.m_aucMessageData[7];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeFloatReadFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float fData, float* pfData, unsigned char* pucData1, unsigned char* pucData2)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clData.fData = fData;
	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clWrite.m_ucMessageLength = 6;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeFloatReadUnsignedLongChars: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeFloatReadUnsignedLongChars: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeFloatReadUnsignedLongChars: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*pfData = clData.fData;
	*pucData1 = clRead.m_aucMessageData[6];
	*pucData2 = clRead.m_aucMessageData[7];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

int CProtocolDevice::writeFloatShortReadFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float fData, short iData, float* pfData, unsigned char* pucData1, unsigned char* pucData2)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	clWrite.m_uiMessageId = MSGID_SET + iModuleId;
	clWrite.m_aucMessageData[0] = iCommandId; 
	clWrite.m_aucMessageData[1] = iParameterId;
	clData.fData = fData;
	clWrite.m_aucMessageData[2] = clData.aucData[0];
	clWrite.m_aucMessageData[3] = clData.aucData[1];
	clWrite.m_aucMessageData[4] = clData.aucData[2];
	clWrite.m_aucMessageData[5] = clData.aucData[3];
	clData.aiData[0] = iData;
	clWrite.m_aucMessageData[6] = clData.aucData[0];
	clWrite.m_aucMessageData[7] = clData.aucData[1];
	clWrite.m_ucMessageLength = 8;
	clWrite.m_iModuleId = iModuleId;
	clRead = clWrite;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_ACK + iModuleId)
		{
			debug(1,"writeFloatShortReadUnsignedLongChars: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_ACK + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != iCommandId)
		{
			debug(1,"writeFloatShortReadUnsignedLongChars: wrong command ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[1] != iParameterId)
		{
			debug(1,"writeFloatShortReadUnsignedLongChars: wrong parameter ID");
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	*pfData = clData.fData;
	*pucData1 = clRead.m_aucMessageData[6];
	*pucData2 = clRead.m_aucMessageData[7];
	LeaveCriticalSection(&m_csDevice);

	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CProtocolDevice::CProtocolDevice() //: m_iEMSModuleCount(0), m_iEMSModuleCountMax(31)
{
}

CProtocolDevice::CProtocolDevice(const CProtocolDevice& rclProtocolDevice)
{
	error(-1, "Sorry constructor is not implemented");
}

CProtocolDevice::~CProtocolDevice()
{
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CProtocolDevice& CProtocolDevice::operator=(const CProtocolDevice& rclProtocolDevice)
{
	error(-1, "Sorry operator= is not implemented");
	return *this;
}

// ========================================================================== ;
//                                                                            ;
// ---- query functions ----------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

int CProtocolDevice::getDataDLR_FTS(std::vector<float>& rafData, long* piState)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}

	rafData.resize(6);
	clWrite.m_uiMessageId = MSGID_DLR_DATA_GET;
	clWrite.m_aucMessageData[0] = CMDID_DLR_DATA_GET;
	clWrite.m_aucMessageData[1] = PARID_DLR_DATA_GET;
	clWrite.m_ucMessageLength = 2;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong 1. readDevice ErrorCode %i", m_iErrorState);
			clearReadQueue();
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_DLR_DATA_ACK)
		{
			debug(1,"getDataDLR_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_DLR_DATA_ACK );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != CMDID_DLR_DATA_ACK0)
		{
			debug(1,"getDataDLR_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_DLR_DATA_ACK0);
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	clData.aucData[0] = clRead.m_aucMessageData[1];
	clData.aucData[1] = clRead.m_aucMessageData[2];
	clData.aucData[2] = clRead.m_aucMessageData[3];
	clData.aucData[3] = clRead.m_aucMessageData[4];
	rafData[0] = clData.fData;
	clData.aucData[0] = clRead.m_aucMessageData[5];
	clData.aucData[1] = clRead.m_aucMessageData[6];
	clData.aucData[2] = clRead.m_aucMessageData[7];

	bRecieved = false;
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong 2. readDevice ErrorCode %i", m_iErrorState);
			clearReadQueue();
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_DLR_DATA_ACK)
		{
			debug(1,"getDataDLR_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_DLR_DATA_ACK );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != CMDID_DLR_DATA_ACK1)
		{
			debug(1,"getDataDLR_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_DLR_DATA_ACK1);
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	clData.aucData[3] = clRead.m_aucMessageData[1];
	rafData[1] = clData.fData;
	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	rafData[2] = clData.fData;
	clData.aucData[0] = clRead.m_aucMessageData[6];
	clData.aucData[1] = clRead.m_aucMessageData[7];

	bRecieved = false;
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong 3. readDevice ErrorCode %i", m_iErrorState);
			clearReadQueue();
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_DLR_DATA_ACK)
		{
			debug(1,"getDataDLR_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_DLR_DATA_ACK );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != CMDID_DLR_DATA_ACK2)
		{
			debug(1,"getDataDLR_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_DLR_DATA_ACK2);
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	clData.aucData[2] = clRead.m_aucMessageData[1];
	clData.aucData[3] = clRead.m_aucMessageData[2];
	rafData[3] = clData.fData;
	clData.aucData[0] = clRead.m_aucMessageData[3];
	clData.aucData[1] = clRead.m_aucMessageData[4];
	clData.aucData[2] = clRead.m_aucMessageData[5];
	clData.aucData[3] = clRead.m_aucMessageData[6];
	rafData[4] = clData.fData;
	clData.aucData[0] = clRead.m_aucMessageData[7];

	bRecieved = false;
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			//warning("wrong 4. readDevice ErrorCode %i", m_iErrorState);
			clearReadQueue();
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_DLR_DATA_ACK)
		{
			debug(1,"getDataDLR_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_DLR_DATA_ACK );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != CMDID_DLR_DATA_ACK3)
		{
			debug(1,"getDataDLR_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_DLR_DATA_ACK3);
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	clData.aucData[1] = clRead.m_aucMessageData[1];
	clData.aucData[2] = clRead.m_aucMessageData[2];
	clData.aucData[3] = clRead.m_aucMessageData[3];
	rafData[5] = clData.fData;
	clData.aucData[0] = clRead.m_aucMessageData[4];
	clData.aucData[1] = clRead.m_aucMessageData[5];
	clData.aucData[2] = clRead.m_aucMessageData[6];
	clData.aucData[3] = clRead.m_aucMessageData[7];
	*piState = clData.iData;

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::getDataSCHUNK_FTC(int iModuleId, int iChannelTypeId, std::vector<float>& rafData, short* piState)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 1 || iModuleId > MAX_SCHUNK )
	{
		warning("bad SCHUNK module ID %i", iModuleId);
		m_iErrorState = ERRID_DEV_WRONGSCHUNKMODULEID;
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	rafData.resize(6);
	clWrite.m_uiMessageId = MSGID_SCHUNK_SEND + iModuleId;
	if(iChannelTypeId == 0)
		clWrite.m_aucMessageData[0] = CMDID_SCHUNK_GET_FTI;
	else if(iChannelTypeId == 1)
		clWrite.m_aucMessageData[0] = CMDID_SCHUNK_GET_TRI;
	else
		clWrite.m_aucMessageData[0] = CMDID_SCHUNK_GET_TEI;
	clWrite.m_ucMessageLength = 1;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			warning("wrong 1. readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_SCHUNK_RECV + iModuleId)
		{
			debug(1,"getDataSCHUNK_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_SCHUNK_RECV + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			bRecieved = false;
		}
		if(iChannelTypeId == 0)
		{
			if(clRead.m_aucMessageData[0] != CMDID_SCHUNK_GET_FTI + 1)
			{
				debug(1,"getDataSCHUNK_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_SCHUNK_GET_FTI + 1);
				debug(2,"write MessageId %x", clWrite.m_uiMessageId);
				debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
				debug(2,"read MessageId %x", clRead.m_uiMessageId);
				debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
				bRecieved = false;
			}
		}
		else if(iChannelTypeId == 1)
		{
			if(clRead.m_aucMessageData[0] != CMDID_SCHUNK_GET_TRI + 1)
			{
				debug(1,"getDataSCHUNK_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_SCHUNK_GET_TRI + 1);
				debug(2,"write MessageId %x", clWrite.m_uiMessageId);
				debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
				debug(2,"read MessageId %x", clRead.m_uiMessageId);
				debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
				bRecieved = false;
			}
		}
		else
		{
			if(clRead.m_aucMessageData[0] != CMDID_SCHUNK_GET_TEI + 1)
			{
				debug(1,"getDataSCHUNK_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_SCHUNK_GET_TEI + 1);
				debug(2,"write MessageId %x", clWrite.m_uiMessageId);
				debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
				debug(2,"read MessageId %x", clRead.m_uiMessageId);
				debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
				bRecieved = false;
			}
		}
	}
	while(!bRecieved);
	clData.aucData[0] = clRead.m_aucMessageData[1];
	clData.aucData[1] = clRead.m_aucMessageData[2];
	clData.aucData[2] = clRead.m_aucMessageData[3];
	clData.aucData[3] = clRead.m_aucMessageData[4];
	if(iChannelTypeId == 0)
	{
		rafData[0] = clData.aiData[0] / 32.;
		rafData[1] = clData.aiData[1] / 32.;
	}
	else if(iChannelTypeId == 1)
	{
		rafData[0] = clData.aiData[0] / 4096000.;
		rafData[1] = clData.aiData[1] / 4096000.;
	}
	else
	{
		rafData[0] = clData.aiData[0];
		rafData[1] = clData.aiData[1];
	}
	clData.aucData[0] = clRead.m_aucMessageData[5];
	clData.aucData[1] = clRead.m_aucMessageData[6];
	clData.aucData[2] = clRead.m_aucMessageData[7];
	if(iChannelTypeId == 0)
	{
		rafData[2] = clData.aiData[0] / 32.;
	}
	else if(iChannelTypeId == 1)
	{
		rafData[2] = clData.aiData[0] / 4096000.;
	}
	else
	{
		rafData[2] = clData.aiData[0];
	}

	bRecieved = false;
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			warning("wrong 2. readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_SCHUNK_RECV + iModuleId)
		{
			debug(1,"getDataSCHUNK_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_SCHUNK_RECV + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != 0x01)
		{
			debug(1,"getDataSCHUNK_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], 0x01);
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	clData.aucData[3] = clRead.m_aucMessageData[1];
	if(iChannelTypeId == 0)
	{
		rafData[3] = clData.aiData[1] / 1024.;
	}
	else if(iChannelTypeId == 1)
	{
		rafData[3] = clData.aiData[1] / 65536.;
	}
	else
	{
		rafData[3] = clData.aiData[1];
	}
	clData.aucData[0] = clRead.m_aucMessageData[2];
	clData.aucData[1] = clRead.m_aucMessageData[3];
	clData.aucData[2] = clRead.m_aucMessageData[4];
	clData.aucData[3] = clRead.m_aucMessageData[5];
	if(iChannelTypeId == 0)
	{
		rafData[4] = clData.aiData[0] / 1024.;
		rafData[5] = clData.aiData[1] / 1024.;
	}
	else if(iChannelTypeId == 1)
	{
		rafData[4] = clData.aiData[0] / 65536.;
		rafData[5] = clData.aiData[1] / 65536.;
	}
	else
	{
		rafData[4] = clData.aiData[0];
		rafData[5] = clData.aiData[1];
	}
	clData.aucData[0] = clRead.m_aucMessageData[6];
	clData.aucData[1] = clRead.m_aucMessageData[7];

	*piState = clData.aiData[0];

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::getDataMP55_IO(int iModuleId, float* pfData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;

	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > MAX_MP55 )
	{
		warning("bad MP55 module ID %i", iModuleId);
		m_iErrorState = ERRID_DEV_WRONGMP55MODULEID;
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	*pfData = 0;

	clWrite.m_uiMessageId = MSGID_MP55_SEND + iModuleId;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = 0x40;
	clWrite.m_aucMessageData[1] = 0x01;
	clWrite.m_aucMessageData[2] = 0x30;
	clWrite.m_aucMessageData[3] = 0x01;
	clWrite.m_aucMessageData[4] = 0x00;
	clWrite.m_aucMessageData[5] = 0x00;
	clWrite.m_aucMessageData[6] = 0x00;
	clWrite.m_aucMessageData[7] = 0x00;
	clWrite.m_ucMessageLength = 8;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_MP55_RECV + iModuleId)
		{
			debug(1,"getDataMP55_IO: received CAN-ID %x, expected %x", clRead.m_uiMessageId, 0x580 + iModuleId);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[4];
	clData.aucData[1] = clRead.m_aucMessageData[5];
	clData.aucData[2] = clRead.m_aucMessageData[6];
	clData.aucData[3] = clRead.m_aucMessageData[7];

	*pfData = clData.fData;

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::getDataMP55_IO_fast(int iModuleId, float* pfData)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;

	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > MAX_MP55 )
	{
		warning("bad MP55 module ID %i", iModuleId);
		m_iErrorState = ERRID_DEV_WRONGMP55MODULEID;
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	*pfData = 0;

	// Send SYNC
	clWrite.m_uiMessageId = 0x80;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = iModuleId;
	clWrite.m_ucMessageLength = 1;

	m_iErrorState = writeDevice(clWrite);

	// Read PDO
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			LeaveCriticalSection(&m_csDevice);
//			printf( "read PDO: %d\n", m_iErrorState );
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != 0x180 + iModuleId)
		{
//			printf("getDataMP55_IO: received CAN-ID %x, expected %x, len= %d\n", clRead.m_uiMessageId, 0x180 + iModuleId, clRead.m_ucMessageLength);
//			for( int i = 0; i < clRead.m_ucMessageLength; i++ )
//				printf( "data[%d]=%x\n", i, clRead.m_aucMessageData[i] );
			bRecieved = false;
		}
		Sleep( 1 );
	}
	while(!bRecieved);
//	printf( "getData PDO: error state %x\n", m_iErrorState );
//	printf( "getData PDO: received CAN-ID %x, expected %x\n", clRead.m_uiMessageId, 0x180 + iModuleId);

	clData.aucData[0] = clRead.m_aucMessageData[0];
	clData.aucData[1] = clRead.m_aucMessageData[1];
	clData.aucData[2] = clRead.m_aucMessageData[2];
	clData.aucData[3] = clRead.m_aucMessageData[3];

	//*pfData = ((float)clData.iData)/100000.0;
	*pfData = clData.fData;

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::getCanOpenRawAbsEnc(int iModuleId, short* piValue)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;

	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > MAX_MP55 )
	{
		warning("bad CanOpen module ID %i", iModuleId);
		m_iErrorState = ERRID_DEV_WRONGMP55MODULEID;
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	*piValue = 0;

	clWrite.m_uiMessageId = MSGID_MP55_SEND + iModuleId;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = 0x40;
	clWrite.m_aucMessageData[1] = 0x04;
	clWrite.m_aucMessageData[2] = 0x60;
	clWrite.m_aucMessageData[3] = 0x00;
	clWrite.m_aucMessageData[4] = 0x00;
	clWrite.m_aucMessageData[5] = 0x00;
	clWrite.m_aucMessageData[6] = 0x00;
	clWrite.m_aucMessageData[7] = 0x00;
	clWrite.m_ucMessageLength = 4;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			printf( "wrong readDevice %d\n", m_iErrorState );
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_MP55_RECV + iModuleId)
		{
			printf("getCanOpenRawAbsEnc: received CAN-ID %lu, expected %x \n", clRead.m_uiMessageId, 0x580 + iModuleId);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clData.aucData[0] = clRead.m_aucMessageData[4];
	clData.aucData[1] = clRead.m_aucMessageData[5];
	clData.aucData[2] = clRead.m_aucMessageData[6];
	clData.aucData[3] = clRead.m_aucMessageData[7];

	*piValue = clData.aiData[0];

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}


// ========================================================================== ;
//                                                                            ;
// ---- modify functions ---------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

int CProtocolDevice::setNullSCHUNK_FTC(int iModuleId, short* piState)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 1 || iModuleId > MAX_SCHUNK )
	{
		warning("bad SCHUNK module ID %i", iModuleId);
		m_iErrorState = ERRID_DEV_WRONGSCHUNKMODULEID;
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	clWrite.m_uiMessageId = MSGID_SCHUNK_SEND + iModuleId;
	clWrite.m_aucMessageData[0] = CMDID_SCHUNK_SET_NULL;
	clWrite.m_ucMessageLength = 1;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			warning("wrong 1. readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_SCHUNK_RECV + iModuleId)
		{
			debug(1,"setNullSCHUNK_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_SCHUNK_RECV + iModuleId );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			bRecieved = false;
		}
		if(clRead.m_aucMessageData[0] != CMDID_SCHUNK_SET_NULL + 1)
		{
			debug(1,"setNullSCHUNK_FTS: wrong command ID %x, expected %x", clRead.m_aucMessageData[0], CMDID_SCHUNK_SET_NULL + 1);
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			bRecieved = false;
		}
	}
	while(!bRecieved);
	if(clRead.m_aucMessageData[1] != 'O' || clRead.m_aucMessageData[2] != 'K')
	{
		debug(1,"setNullSCHUNK_FTS: wrong answer '%c%c', expected 'OK'", clRead.m_aucMessageData[1], clRead.m_aucMessageData[2]);
	}
	clData.aucData[0] = clRead.m_aucMessageData[3];
	clData.aucData[1] = clRead.m_aucMessageData[4];

	*piState = clData.aiData[0];
	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::setTaraMP55_IO(int iModuleId, float fTara)
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;

	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > MAX_MP55 )
	{
		warning("bad MP55 module ID %i", iModuleId);
		m_iErrorState = ERRID_DEV_WRONGMP55MODULEID;
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	clData.iData = fTara * 1000.0;

	clWrite.m_uiMessageId = 0x600 + iModuleId;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = 0x23;
	clWrite.m_aucMessageData[1] = 0x80;
	clWrite.m_aucMessageData[2] = 0x21;
	clWrite.m_aucMessageData[3] = 0x01;
	clWrite.m_aucMessageData[4] = clData.aucData[0];
	clWrite.m_aucMessageData[5] = clData.aucData[1];
	clWrite.m_aucMessageData[6] = clData.aucData[2];
	clWrite.m_aucMessageData[7] = clData.aucData[3];
	clWrite.m_ucMessageLength = 8;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}
	
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			warning("wrong readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != 0x580 + iModuleId)
		{
			debug(1,"setTaraMP55_FS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, 0x580 + iModuleId);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::setInitMP55_IO_fast( int iModuleId )
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;

	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;
	static CProtocolData clData;

	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > MAX_MP55 )
	{
		warning("bad MP55 module ID %i", iModuleId);
		m_iErrorState = ERRID_DEV_WRONGMP55MODULEID;
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	// Init MP55: Stop Operational Mode
	clWrite.m_uiMessageId = 0x000;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = 0x80;
	clWrite.m_aucMessageData[1] = 0x00;
	clWrite.m_ucMessageLength = 2;

	m_iErrorState = writeDevice(clWrite);

//	printf( "written Start Operational\n" );
	Sleep( 20 );

	// Init MP55: Set TPDO on SYNC (object 1800)
	clWrite.m_uiMessageId = 0x600 + iModuleId;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = 0x2F;
	clWrite.m_aucMessageData[1] = 0x00;
	clWrite.m_aucMessageData[2] = 0x18;
	clWrite.m_aucMessageData[3] = 0x02;
	clWrite.m_aucMessageData[4] = 0x01;
	clWrite.m_aucMessageData[5] = 0x00;
	clWrite.m_aucMessageData[6] = 0x00;
	clWrite.m_aucMessageData[7] = 0x00;
	clWrite.m_ucMessageLength = 8;

	m_iErrorState = writeDevice(clWrite);

	int end = 0;
	do
	{	
		m_iErrorState = readDevice(clRead);
		bRecieved = true;
		if(clRead.m_uiMessageId != 0x580 + iModuleId)
		{
//			printf( "setTaraMP55_FS: received CAN-ID %x, expected %x\n", clRead.m_uiMessageId, 0x580 + iModuleId);
			bRecieved = false;
		}
		else
		{	if( clRead.m_aucMessageData[0] != 0x60 )
				bRecieved = false;
			else 
			{	if( clRead.m_aucMessageData[1] != 0x00 )
					bRecieved = false;
				else
				{ if( clRead.m_aucMessageData[1] != 0x18 )
						bRecieved = false;
				}
			}	
		}
		end++;
	}
	while(!bRecieved && end < 10);

//	printf( "setTara obj 1800: received CAN-ID %x, expected %x\n", clRead.m_uiMessageId, 0x580 + iModuleId);
//	for( int i = 0; i < clRead.m_ucMessageLength; i++ )
//		printf( "data[%d]=%x\n", i, clRead.m_aucMessageData[i] );


	// Init MP55: Set TPDO to one message (object 1801)
	clWrite.m_uiMessageId = 0x600 + iModuleId;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = 0x2F;
	clWrite.m_aucMessageData[1] = 0x01;
	clWrite.m_aucMessageData[2] = 0x18;
	clWrite.m_aucMessageData[3] = 0x02;
	clWrite.m_aucMessageData[4] = 0xFF;
	clWrite.m_aucMessageData[5] = 0x00;
	clWrite.m_aucMessageData[6] = 0x00;
	clWrite.m_aucMessageData[7] = 0x00;
	clWrite.m_ucMessageLength = 8;

	m_iErrorState = writeDevice(clWrite);

//	printf( "written obj 1801\n" );
	Sleep( 10 );
	end = 0;
	do
	{	
		m_iErrorState = readDevice(clRead);
		bRecieved = true;
		if(clRead.m_uiMessageId != 0x580 + iModuleId)
		{
//			printf( "setTaraMP55_FS: received CAN-ID %x, expected %x\n", clRead.m_uiMessageId, 0x580 + iModuleId);
			bRecieved = false;
		}
		else
		{	if( clRead.m_aucMessageData[0] != 0x60 )
				bRecieved = false;
			else 
			{	if( clRead.m_aucMessageData[1] != 0x01 )
					bRecieved = false;
				else
				{ if( clRead.m_aucMessageData[1] != 0x18 )
						bRecieved = false;
				}
			}	
		}
		end++;
	}
	while(!bRecieved && end < 10);

//	printf( "setTara obj 1801: received CAN-ID %x, expected %x\n", clRead.m_uiMessageId, 0x580 + iModuleId);
//	for( i = 0; i < clRead.m_ucMessageLength; i++ )
//		printf( "data[%d]=%x\n", i, clRead.m_aucMessageData[i] );

	Sleep( 10 );

	// Init MP55: Set to Operational Mode
	clWrite.m_uiMessageId = 0x000;
	clWrite.m_bRTRFlag = false;
	clWrite.m_aucMessageData[0] = 0x01;
	clWrite.m_aucMessageData[1] = 0x00;
	clWrite.m_ucMessageLength = 2;

	m_iErrorState = writeDevice(clWrite);

//	printf( "written Start Operational\n" );
	Sleep( 10 );

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
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

int CProtocolDevice::initDLR_FTS()
{
	EnterCriticalSection(&m_csDevice);
	m_iErrorState = 0;
	bool bRecieved = false;
	static CProtocolMessage clWrite, clRead;

	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = setMessageId(MSGID_DLR_INIT_ACK);
	if(m_iErrorState != 0)
	{
		warning("wrong setMessageId");
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}		
	m_iErrorState = setMessageId(MSGID_DLR_DATA_ACK);
	if(m_iErrorState != 0)
	{
		warning("wrong setMessageId");
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}		
	clWrite.m_uiMessageId = MSGID_DLR_INIT_0;
	clWrite.m_aucMessageData[0] = 0;
	clWrite.m_aucMessageData[1] = 0x4b;
	clWrite.m_aucMessageData[2] = 0x3;
	clWrite.m_aucMessageData[3] = 0x1;
	clWrite.m_aucMessageData[4] = 0x3;
	clWrite.m_aucMessageData[5] = 0;
	clWrite.m_ucMessageLength = 6;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			warning("wrong MSGID_DLR_INIT_0 readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_DLR_INIT_ACK)
		{
			debug(1,"initDLR_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_DLR_INIT_ACK );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clWrite.m_uiMessageId = MSGID_DLR_INIT_1;
	clWrite.m_aucMessageData[0] = 0x40;
	clWrite.m_aucMessageData[1] = 0x10;
	clWrite.m_aucMessageData[2] = 0x5;
	clWrite.m_aucMessageData[3] = 0x1;
	clWrite.m_aucMessageData[4] = 0x9;
	clWrite.m_aucMessageData[5] = 0;
	clWrite.m_aucMessageData[6] = 0;
	clWrite.m_ucMessageLength = 7;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	bRecieved = false;
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			warning("wrong MSGID_DLR_INIT_1 readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_DLR_INIT_ACK)
		{
			debug(1,"initDLR_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_DLR_INIT_ACK );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	clWrite.m_uiMessageId = MSGID_DLR_INIT_2;
	clWrite.m_aucMessageData[0] = 0;
	clWrite.m_aucMessageData[1] = 0x10;
	clWrite.m_aucMessageData[2] = 0x5;
	clWrite.m_aucMessageData[3] = 0x2;
	clWrite.m_aucMessageData[4] = 0x9;
	clWrite.m_aucMessageData[5] = 0x10;
	clWrite.m_aucMessageData[6] = 0x27;
	clWrite.m_ucMessageLength = 7;

	m_iErrorState = writeDevice(clWrite);
	if(m_iErrorState != 0)
	{
		warning("wrong writeDevice ErrorCode %i", m_iErrorState);
		LeaveCriticalSection(&m_csDevice);
		return m_iErrorState;
	}

	bRecieved = false;
	do
	{	
		m_iErrorState = readDevice(clRead);
		if(m_iErrorState != 0)
		{
			warning("wrong MSGID_DLR_INIT_2 readDevice ErrorCode %i", m_iErrorState);
			LeaveCriticalSection(&m_csDevice);
			return m_iErrorState;
		}
		bRecieved = true;
		if(clRead.m_uiMessageId != MSGID_DLR_INIT_ACK)
		{
			debug(1,"initDLR_FTS: received CAN-ID %x, expected %x", clRead.m_uiMessageId, MSGID_DLR_INIT_ACK );
			debug(2,"write MessageId %x", clWrite.m_uiMessageId);
			debug(2,"write CommandId %x", clWrite.m_aucMessageData[0]);
			debug(2,"write ParameterId %x", clWrite.m_aucMessageData[1]);
			debug(2,"read MessageId %x", clRead.m_uiMessageId);
			debug(2,"read CommandId %x", clRead.m_aucMessageData[0]);
			debug(2,"read ParameterId %x", clRead.m_aucMessageData[1]);
			bRecieved = false;
		}
	}
	while(!bRecieved);

	LeaveCriticalSection(&m_csDevice);
	return m_iErrorState;
}

int CProtocolDevice::printMessage(CProtocolMessage & rclProtocolMessage, bool read)
{

        char readWrite[10];
        if (read)
          sprintf(readWrite,"read");
        else
          sprintf(readWrite,"write");

        if (rclProtocolMessage.m_ucMessageLength == 8)
        {
                debug(2,"%s CAN message Id 0x%02x, Command Id 0x%02x, ParameterId 0x%02x, Data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",readWrite,
                                                                                rclProtocolMessage.m_uiMessageId,
                                                                                rclProtocolMessage.m_aucMessageData[0],
                                                                                rclProtocolMessage.m_aucMessageData[1],
                                                                                rclProtocolMessage.m_aucMessageData[2],
                                                                                rclProtocolMessage.m_aucMessageData[3],
                                                                                rclProtocolMessage.m_aucMessageData[4],
                                                                                rclProtocolMessage.m_aucMessageData[5],
                                                                                rclProtocolMessage.m_aucMessageData[6],
                                                                                rclProtocolMessage.m_aucMessageData[7]); 
        }
        else if (rclProtocolMessage.m_ucMessageLength == 7)
        {
                debug(2,"%s CAN message Id 0x%02x, Command Id 0x%02x, ParameterId 0x%02x, Data: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",readWrite,
                                                                                rclProtocolMessage.m_uiMessageId,
                                                                                rclProtocolMessage.m_aucMessageData[0],
                                                                                rclProtocolMessage.m_aucMessageData[1],
                                                                                rclProtocolMessage.m_aucMessageData[2],
                                                                                rclProtocolMessage.m_aucMessageData[3],
                                                                                rclProtocolMessage.m_aucMessageData[4],
                                                                                rclProtocolMessage.m_aucMessageData[5],
                                                                                rclProtocolMessage.m_aucMessageData[6]); 
        }
        else if (rclProtocolMessage.m_ucMessageLength == 6)
        {
                debug(2,"%s CAN message Id 0x%02x, Command Id 0x%02x, ParameterId 0x%02x, Data: 0x%02x 0x%02x 0x%02x 0x%02x",readWrite,
                                                                                rclProtocolMessage.m_uiMessageId,
                                                                                rclProtocolMessage.m_aucMessageData[0],
                                                                                rclProtocolMessage.m_aucMessageData[1],
                                                                                rclProtocolMessage.m_aucMessageData[2],
                                                                                rclProtocolMessage.m_aucMessageData[3],
                                                                                rclProtocolMessage.m_aucMessageData[4],
                                                                                rclProtocolMessage.m_aucMessageData[5]); 
        }
        else if (rclProtocolMessage.m_ucMessageLength == 5)
        {
                debug(2,"%s CAN message Id 0x%02x, Command Id 0x%02x, ParameterId 0x%02x, Data: 0x%02x 0x%02x 0x%02x",readWrite,
                                                                                rclProtocolMessage.m_uiMessageId,
                                                                                rclProtocolMessage.m_aucMessageData[0],
                                                                                rclProtocolMessage.m_aucMessageData[1],
                                                                                rclProtocolMessage.m_aucMessageData[2],
                                                                                rclProtocolMessage.m_aucMessageData[3],
                                                                                rclProtocolMessage.m_aucMessageData[4]); 
        }
        else if (rclProtocolMessage.m_ucMessageLength == 4)
        {
                debug(2,"%s CAN message Id 0x%02x, Command Id 0x%02x, ParameterId 0x%02x, Data: 0x%02x 0x%02x",readWrite,
                                                                                rclProtocolMessage.m_uiMessageId,
                                                                                rclProtocolMessage.m_aucMessageData[0],
                                                                                rclProtocolMessage.m_aucMessageData[1],
                                                                                rclProtocolMessage.m_aucMessageData[2],
                                                                                rclProtocolMessage.m_aucMessageData[3]);
        }
        else if (rclProtocolMessage.m_ucMessageLength == 3)
        {
                debug(2,"%s CAN message Id 0x%02x, Command Id 0x%02x, ParameterId 0x%02x, Data: 0x%02x",readWrite,
                                                                                rclProtocolMessage.m_uiMessageId,
                                                                                rclProtocolMessage.m_aucMessageData[0],
                                                                                rclProtocolMessage.m_aucMessageData[1],
                                                                                rclProtocolMessage.m_aucMessageData[2]);
        }
        else if (rclProtocolMessage.m_ucMessageLength == 2)
        {
                debug(2,"%s CAN message Id 0x%02x, Command Id 0x%02x, ParameterId 0x%02x",readWrite,
                                                                                 rclProtocolMessage.m_uiMessageId,
                                                                                 rclProtocolMessage.m_aucMessageData[0],
                                                                                 rclProtocolMessage.m_aucMessageData[1]);
        }

        return 0;


}
