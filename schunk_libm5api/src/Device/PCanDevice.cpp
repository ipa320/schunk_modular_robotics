
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


#include "PCanDevice.h"
#include <errno.h>
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

int CPCanDevice::getDeviceError(int iErrorState)
{
        int error = ERRID_DEV_WRITEERROR;

	if(((unsigned int)iErrorState & 0xFFFF) & CAN_ERR_QRCVEMPTY)
	{
		warning("receive queue is empty");
		error= ERRID_DEV_WRITEERROR;
	}
	if(iErrorState & CAN_ERR_OVERRUN)
	{
		warning("receive buffer overrun");
		error= ERRID_DEV_READERROR;
	}
	if(iErrorState & CAN_ERR_XMTFULL)
	{
		warning("transmit buffer full");
		error =  ERRID_DEV_WRITEERROR;
	}
        if(iErrorState & CAN_ERR_BUSOFF)
	{
		warning("CAN_ERR_OFF_BUS");
		error =  ERRID_DEV_READERROR;
	}
	if(iErrorState & CAN_ERR_ILLPARAMTYPE )
	{
		warning("CAN_ERR_ILLPARAMTYPE");
		error =  ERRID_DEV_READERROR;
	}
	/*
	 * no corresponding errors in pcan.h found
	 *
	else if(iErrorState == NTCAN_MESSAGE_LOST)
	{
		warning("NTCAN_MESSAGE_LOST");
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
	*/
	if(((unsigned int)iErrorState & 0xFFFF) & CAN_ERR_QXMTFULL)
	{
		warning("transmit queue full");
		error =  ERRID_DEV_WRITEERROR;
	}
	if(iErrorState & CAN_ERR_BUSLIGHT)
	{
		warning("bus error");
		error =  ERRID_DEV_WRITEERROR;
	}
	if(((unsigned int)iErrorState & 0xFFFF) & CAN_ERR_BUSHEAVY)
	{
		warning("bus error");
		error =  ERRID_DEV_WRITEERROR;
	}
	if(((unsigned int)iErrorState & 0xFFFF) & CAN_ERR_RESOURCE)
	{
		warning("can't create resource");
		error =  ERRID_DEV_WRITEERROR;
	}

	return error;
}

//ok
int CPCanDevice::setBaudRate(unsigned char iBaudRate)
{
	m_uiBaudRate = (unsigned long) iBaudRate;
	return setBaudRate();
}

//ok
int CPCanDevice::setBaudRate()
{
	/*
	 * baud rate must be set when initializing can device!
	 *
	 * */

        debug(0,"entering CPCanDevice::setBaudRate()...\n");
	warning("PCan Device must be reset to set the new baud rate!\n");
	
	int iRetVal = 0;
	m_iErrorState = 0;

	switch( m_iBaudRate )
	{
		case 125:
			m_uiBaudRate=CAN_BAUD_125K;	// 125k
			break;
		case 250:
			m_uiBaudRate=CAN_BAUD_250K;	// 250k
			break;
		case 500:
			m_uiBaudRate=CAN_BAUD_500K;	// 500k
			break;
		case 1000:
			m_uiBaudRate=CAN_BAUD_1M;	// 1000k
			break;
		default:
			m_uiBaudRate=CAN_BAUD_250K;	// 250k
			break;
	}

	
	if (m_bInitFlag)
	{
		CAN_Close(m_handle);
	}
	iRetVal = init(m_uiBaudRate);
        debug(0,"InitFlag set to %d\n",m_bInitFlag);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("can set baudrate 0x%x failed Errorcode: %d", m_uiBaudRate, iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}	
        else
        {
          debug(0,"PCanDevice: setting baud rate to %d\n",m_iBaudRate);
        }
	return m_iErrorState;
}

//ok
int CPCanDevice::setMessageId(unsigned long uiMessageId)
{
	int iRetVal = 0;
	m_iErrorState = 0;
	//iRetVal = canIdAdd(m_hDevice, uiMessageId);
	iRetVal = CAN_MsgFilter(m_handle, uiMessageId ,uiMessageId,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}
	return m_iErrorState;
}

//ok
int CPCanDevice::clearReadQueue()
{
	int iRetVal = 0;
	TPCANRdMsg TPCMsg;
	
        debug(0,"entering CPCanDevice::clearReadQueue()...\n");
        TPCMsg.Msg.LEN = 8;
	TPCMsg.Msg.MSGTYPE = 0;
	TPCMsg.Msg.ID = 0;

	m_iErrorState = 0;
	do
	{	
		//iRetVal = canRead(m_hDevice, &clESDProtocolMessage, &iNumberOfMessages, NULL);			
	        debug(0,"Trying to read messages ...");
                iRetVal = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, m_uiTimeOut);
                debug(0," 0x%04x\n",iRetVal);

	}while( iRetVal != CAN_ERR_QRCVEMPTY ) ;
	
	return m_iErrorState;
}


//ok
int CPCanDevice::reinit(unsigned char ucBaudRateId)
{

	int  iRetVal = 0;
	
	m_iErrorState = 0;
	if(!m_bInitFlag)
	{
		warning("reinit:device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	iRetVal = setBaudRate(ucBaudRateId);

	/* done in setBaudRate
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
	if(iRetVal != CAN_ERR_OK)
	{
		warning("can close failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_EXITERROR;
	}
	iRetVal = canClose(m_hSyncDevice);
	if(iRetVal != CAN_ERR_OK)
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
	if(iRetVal != CAN_ERR_OK)
	{
		warning("can open failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}
	*/

	//iRetVal = canIdAdd(m_hDevice, (MSGID_ACK + i));
	iRetVal = CAN_MsgFilter(m_handle, MSGID_ACK ,MSGID_ACK+m_iModuleCountMax,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}
	//iRetVal = canIdAdd(m_hDevice, (MSGID_STATE + i));
	iRetVal = CAN_MsgFilter(m_handle, MSGID_STATE ,MSGID_STATE+m_iModuleCountMax,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hDevice, (MSGID_MP55_RECV + i));
	iRetVal = CAN_MsgFilter(m_handle, MSGID_MP55_RECV ,MSGID_MP55_RECV+MAX_MP55,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hDevice, (0x180 + i));
	iRetVal = CAN_MsgFilter(m_handle, 0x180 ,0x180+MAX_MP55,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hDevice, (MSGID_SCHUNK_RECV + i));
	iRetVal = CAN_MsgFilter(m_handle, MSGID_SCHUNK_RECV ,MSGID_SCHUNK_RECV+MAX_SCHUNK,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hSyncDevice, MSGID_ALL);
	iRetVal = CAN_MsgFilter(m_handle, MSGID_ALL ,MSGID_ALL,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	m_iErrorState = clearReadQueue();
	if(m_iErrorState != 0)
		return m_iErrorState;

	if(m_iErrorState == 0)
        {
                m_bInitFlag = true;
        }

	
	updateModuleIdMap();
	return m_iErrorState;
}



int CPCanDevice::readDevice(CProtocolMessage& rclProtocolMessage)
{
	int iRetVal = 0;
	
	TPCANRdMsg TPCMsg;
	TPCMsg.Msg.LEN = 8;
	TPCMsg.Msg.MSGTYPE = 0;
	TPCMsg.Msg.ID = 0;

	m_iErrorState = 0;
        int no = 0;
        do
        {
                m_iErrorState = 0;
	        iRetVal = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, m_uiTimeOut);
                if (iRetVal == CAN_ERR_OK)
                  break;
                else
                  CAN_Status(m_handle);
                no++;

                m_iErrorState = getDeviceError(iRetVal);
                debug(2,"Read error (%s), attempt %d of %d",strerror(nGetLastError()),no,m_iNoOfRetries+1);
                //sleep(100);
        }while (no <= m_iNoOfRetries);
	if (iRetVal == CAN_ERR_OK)
	{
		rclProtocolMessage.m_uiMessageId = TPCMsg.Msg.ID;
		rclProtocolMessage.m_ucMessageLength = TPCMsg.Msg.LEN;
		memcpy(rclProtocolMessage.m_aucMessageData, TPCMsg.Msg.DATA, rclProtocolMessage.m_ucMessageLength);
                printMessage(rclProtocolMessage,READ);
	}
	else
	{
                //warning("Last Error reported: %s",strerror(nGetLastError()));
		//m_iErrorState = getDeviceError(iRetVal);
		warning("CAN read failed Errorcode: 0x%04x", iRetVal);
//		return m_iErrorState;
	}
        return m_iErrorState;
}


//ok
int CPCanDevice::writeDevice(CProtocolMessage& rclProtocolMessage)
{
	int iRetVal = 0;
	TPCANMsg TPCMsg;
	
	TPCMsg.MSGTYPE = MSGTYPE_STANDARD;
	m_iErrorState = 0;
	TPCMsg.ID = rclProtocolMessage.m_uiMessageId;
        printMessage(rclProtocolMessage,WRITE);
                                                                              
	TPCMsg.LEN =  rclProtocolMessage.m_ucMessageLength;
	if(rclProtocolMessage.m_bRTRFlag)
		TPCMsg.MSGTYPE = MSGTYPE_RTR;
	memcpy(TPCMsg.DATA, rclProtocolMessage.m_aucMessageData, rclProtocolMessage.m_ucMessageLength);
	iRetVal = CAN_Write(m_handle, &TPCMsg);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("can send failed Errorcode:0x%04x", iRetVal);
		m_iErrorState = getDeviceError(iRetVal);
		//return m_iErrorState;
        }
        iRetVal = CAN_Status(m_handle);
        if (iRetVal < 0)
        {
                warning("Last Error reported: %s",strerror(nGetLastError()));
                m_iErrorState = ERRID_DEV_WRITEERROR;
        }

	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CPCanDevice::CPCanDevice() : m_hDevice(0), m_hSyncDevice(0), m_iDeviceId(-1), m_uiBaudRate(0), m_uiQueueSize(128), m_uiTimeOut(3)		// ESD C331
{
	initMessage("CPCanDevice", g_iDebugLevel, g_bDebug, g_bDebugFile);
	m_DeviceName = (char*) malloc(200 * sizeof(char));
	memset(m_DeviceName,0,sizeof(m_DeviceName));
}

CPCanDevice::CPCanDevice(const CPCanDevice& rclPCanDevice)
{
	error(-1, "Sorry constructor is not implemented");
}

CPCanDevice::~CPCanDevice()
{
	free(m_DeviceName);
	if (m_bInitFlag)
          this->exit();
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CPCanDevice& CPCanDevice::operator=(const CPCanDevice& rclPCanDevice)
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

void CPCanDevice::setQueueSize(unsigned short uiQueueSize)
{
	m_uiQueueSize = uiQueueSize;
}

void CPCanDevice::setTimeOut(unsigned long uiTimeOut)
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

//ok
int CPCanDevice::init()
{
	return init(CAN_BAUD_250K);
}
int CPCanDevice::init(unsigned long baudRate)
{
	int iRetVal = CAN_ERR_OK;
        printf("Initializing pcan device ...\n");
	//m_handle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
	m_handle = LINUX_CAN_Open(m_DeviceName,0);

	if (! m_handle)
	{
		// Fatal error
		printf("Error: Cannot open CAN on USB (%s): %s\n", m_DeviceName, strerror(errno));
		iRetVal = -1;
	}
	else
	{
		iRetVal = CAN_Init(m_handle, baudRate, CAN_INIT_TYPE_ST);
	}
	if(iRetVal != CAN_ERR_OK)
	{
		printf("PcanDevice: error in init" );
	}
	else
	{
		printf("PcanDevice, init ok\n" );
                m_bInitFlag = true;
	}
	return iRetVal;
}

/*
 * Not needed for pcan
 */
int CPCanDevice::init(const char* acInitString)
{
	InitializeCriticalSection(&m_csDevice);
	int iRetVal = 0;
	m_uiTimeOut =6;
        m_iNoOfRetries = 10;
	char* pcToken;
	char acString[128];
	int deb = getDebugLevel();
	if (deb > 0)
	{
		printf("CPCanDevice::init: DebugLevel: %d\n",deb);
		printf("writing debug output to file debug.txt!\n");
	}
        debug(0,"entering CPCanDevice::init(const char* acInitString) ...\n");
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
	if( strcmp( pcToken, "PCAN" ) != 0 )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	pcToken = strtok( NULL, "," );
	if( !pcToken )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}


	
	
	m_iDeviceId = atoi(pcToken);
	//std::cout << m_iDeviceId << std::endl;


	strncpy(m_DeviceName,pcToken,12);
	
	/*
	
	if (m_iDeviceId == 0)
	{
		strcpy(m_DeviceName,"/dev/pcan32");
	}
	else if (m_iDeviceId == 1)
	{
		strcpy(m_DeviceName,"/dev/pcan33");
	}
	else if (m_iDeviceId == 14)
	{
		strcpy(m_DeviceName,"/dev/pcan0");
	}
	else if (m_iDeviceId == 15)
	{
		strcpy(m_DeviceName,"/dev/pcan1");
	}
	else if (m_iDeviceId == 16)
	{
		strcpy(m_DeviceName,"/dev/pcan2");
	}
	else if (m_iDeviceId == 17)
	{
		strcpy(m_DeviceName,"/dev/pcan3");
	}
	else
	{
		printf("Warning: currently only support for 2 devices!\n");
	}
	*/
	
	//printf("Device %s: %s\n",pcToken,m_DeviceName);


	pcToken = strtok( NULL, "," );
	if( !pcToken )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	m_iBaudRate = atoi(pcToken);


	try
	{
		m_handle = LINUX_CAN_Open(m_DeviceName,0);
		if (! m_handle)
		{
			// Fatal error
			printf("Error: Cannot open CAN on USB (%s): %s\n", m_DeviceName, strerror(errno));
			iRetVal = -1;
		}
                else
                {
                        printf("PCanDevice successfully opened on %s\n",m_DeviceName);
                }
	}
	catch(...)
	{
			warning("open PCAN device failed no library found");
			m_iErrorState = ERRID_DEV_NOLIBRARY;
			return m_iErrorState;
	}


	iRetVal = CAN_MsgFilter(m_handle, MSGID_ACK ,MSGID_ACK+m_iModuleCountMax,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}
	//iRetVal = canIdAdd(m_hDevice, (MSGID_STATE + i));
	iRetVal = CAN_MsgFilter(m_handle, MSGID_STATE ,MSGID_STATE+m_iModuleCountMax,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hDevice, (MSGID_MP55_RECV + i));
	iRetVal = CAN_MsgFilter(m_handle, MSGID_MP55_RECV ,MSGID_MP55_RECV+MAX_MP55,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hDevice, (0x180 + i));
	iRetVal = CAN_MsgFilter(m_handle, 0x180 ,0x180+MAX_MP55,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hDevice, (MSGID_SCHUNK_RECV + i));
	iRetVal = CAN_MsgFilter(m_handle, MSGID_SCHUNK_RECV ,MSGID_SCHUNK_RECV+MAX_SCHUNK,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
		getDeviceError(iRetVal);
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	//iRetVal = canIdAdd(m_hSyncDevice, MSGID_ALL);
	iRetVal = CAN_MsgFilter(m_handle, MSGID_ALL ,MSGID_ALL,MSGTYPE_STANDARD);
	if(iRetVal != CAN_ERR_OK)
	{
		warning("Can_MsgFilter failed Errorcode: %d", iRetVal);
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
        {
		m_bInitFlag = true;
                debug(0,"PCanDevice:init successfull!\n");
        }
	updateModuleIdMap();

	return m_iErrorState;
}



//ok
int CPCanDevice::exit()
{
	int iRetVal = 0;
	m_iErrorState = 0;

        //printf diagnosis
        //i
        TPDIAG Diag;
        iRetVal = LINUX_CAN_Statistics(m_handle,&Diag);
        debug(0,"PCanDevice: exit():");
        debug(0,"--------------STATISTICS-------------------");
        debug(0,"Total number of reads: %d",Diag.dwReadCounter);
        debug(0,"Total number of writes: %d",Diag.dwWriteCounter);
        debug(0,"Total number of interrupts: %d",Diag.dwIRQcounter);
        debug(0,"Total number of errors: %d",Diag.dwErrorCounter);
        debug(0,"Error flag: 0x%04x",Diag.wErrorFlag);
        
	if(!m_bInitFlag)
	{
		warning("exit:device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	EnterCriticalSection(&m_csDevice);
	iRetVal = CAN_ERR_OK;
	iRetVal = CAN_Close(m_handle);
	if(iRetVal != CAN_ERR_OK)
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

int CPCanDevice::waitForStartMotionAll()
{
	int iRetVal = 0;
	bool bRecieved = false;
	TPCANRdMsg TPCMsg;
	TPCMsg.Msg.LEN = 8;
	TPCMsg.Msg.MSGTYPE = 0;
	TPCMsg.Msg.ID = 0;

	m_iErrorState = 0;
	iRetVal = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, m_uiTimeOut);

	m_iErrorState = 0;

	do
	{	
		//iRetVal = canRead(m_hSyncDevice, &clESDProtocolMessage, &iNumberOfMessages, NULL);			
		iRetVal = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, m_uiTimeOut);
		if(iRetVal != CAN_ERR_OK)
		{
			warning("can read failed Errorcode: 0x%04x", iRetVal);
			m_iErrorState = getDeviceError(iRetVal);
			return m_iErrorState;
		}
		bRecieved = true;
		if(TPCMsg.Msg.ID != MSGID_ALL)
		{
			debug(1, "received CAN-ID %x, expected %x", TPCMsg.Msg.ID, MSGID_ALL);
			bRecieved = false;
		}
		if(TPCMsg.Msg.DATA[0] != CMDID_STARTMOVE)
		{
			debug(1, "wrong command ID");
			bRecieved = false;
		}
	}
	while(!bRecieved);
	return m_iErrorState;
}

