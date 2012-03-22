
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


#include "RS232Device.h"
#if defined (_WIN32)
#endif
#if defined(__LINUX__)
	#include <fcntl.h>
	#include <termios.h>
	#include <unistd.h>
#endif
#if defined (__QNX__)
	#include <fcntl.h>
	#include <termios.h>	
#endif

#define RSID_SENDDAT 			0x04
#define RSID_RECVDAT 			0x08	
#define STX						0x02
#define ETX						0x03
#define DLE						0x10

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

int CRS232Device::getDeviceError(int iErrorState)
{
	m_iErrorState = 0;
	return iErrorState;
}

int CRS232Device::setBaudRate()
{
	m_iErrorState = 0;
	return m_iErrorState;
}

int CRS232Device::setMessageId(unsigned long uiMessageId)
{
	m_iErrorState = 0;
	return m_iErrorState;
}

int CRS232Device::clearReadQueue()
{
	m_iErrorState = 0;
	return m_iErrorState;
}

int CRS232Device::reinit(unsigned char ucBaudRateId)
{
	m_iErrorState = 0;
	return m_iErrorState;
}

int CRS232Device::readDevice(CProtocolMessage& rclProtocolMessage)
{
	unsigned char aucMessageBuffer[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	unsigned char aucReadBuffer[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	unsigned char ucChar;
	unsigned char aucDecodeBuffer[11] = {0,0,0,0,0,0,0,0,0,0,0};
	unsigned char ucMessageLength = 0;
	unsigned int i = 0;
	unsigned int k = 0;
	unsigned int uiMessageBufferIndex = 0;
	unsigned int uiReadBufferIndex = 0;
	double fTime = 0;
	bool bExit = false;
	bool bMessageComplete = false;
	bool bDecodeError = false;
	int iRetVal;
	m_iErrorState = 0;
#if defined(_WIN32)					
	DWORD iErrorCode = 0;
	DWORD iReadLength = 0;
	m_clTimer.start();

	do
	{	
		iRetVal = ReadFile( m_hDevice, &ucChar, 1, &iReadLength, NULL );
		if (iRetVal == 0)
		{	
			void* lpMsgBuf;
			FormatMessage( 
				FORMAT_MESSAGE_ALLOCATE_BUFFER | 
				FORMAT_MESSAGE_FROM_SYSTEM | 
				FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL,
				GetLastError(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
				(LPTSTR) &lpMsgBuf,
				0,
				NULL 
				);
			debug(1,"COMERROR:%s\n",lpMsgBuf);
			warning( "Receive Error. ReadFile failed.\n" );
			ClearCommError( m_hDevice, &iErrorCode, NULL );
			m_iErrorState = ERRID_DEV_READERROR;
			return m_iErrorState;
		}
		if( iReadLength == 1 )
		{	
			aucReadBuffer[uiReadBufferIndex++] = ucChar;
			if( ucChar == ETX )
				bExit = true;
		}
		m_clTimer.stop();
		fTime = m_clTimer.executionTime() * 1000;
		if( fTime > m_uiTimeOut + 10 * m_iModuleCount)
		{	
			bExit = true;
			warning( "Receive Error. Timeout %f. %d bytes received.\n", fTime, uiReadBufferIndex);
			m_iErrorState = ERRID_DEV_READTIMEOUT;
			return m_iErrorState;
		}
	} while( !bExit );

#endif

#if defined(__QNX__)					
	struct timeval clTimeout;
	fd_set fdReadSet;
	clTimeout.tv_sec=0;     // seconds to wait
	clTimeout.tv_usec=m_uiTimeOut * 1000;
	m_clTimer.start();
	ssize_t bytesRead = 0;
	do
	{	
		FD_ZERO( &fdReadSet );
		FD_SET( m_hDevice, &fdReadSet );

		iRetVal=select( FD_SETSIZE, &fdReadSet, NULL, NULL, &clTimeout );
		if(iRetVal < 0)
		{
			warning("Receive Error in select");
			return ERRID_IO_READERROR;
		}
		else if (iRetVal>0 && FD_ISSET(m_hDevice,&fdReadSet))
		{
			bytesRead = read( m_hDevice, &ucChar, 1 );
			if( bytesRead < 0 )
			{	warning( "Receive Error. Read error.\n" );
				m_iErrorState = ERRID_DEV_READERROR;
				return m_iErrorState;
			}
			else if( bytesRead == 1 )
			{	
				aucReadBuffer[uiReadBufferIndex++] = ucChar;
				if( ucChar == ETX )
					bExit = true;
			}
			else if( bytesRead == 0 )
			{
				warning( "Receive Error. Timeout\n");
				m_iErrorState = ERRID_DEV_READTIMEOUT;
				return m_iErrorState;
			}
		}
		m_clTimer.stop();
		fTime = m_clTimer.executionTime() * 1000;
		if( fTime > m_uiTimeOut * m_iModuleCount)
		{	bExit = true;
			warning( "Receive Error. Timeout. %d bytes received.\n", uiReadBufferIndex );
			m_iErrorState = ERRID_DEV_READTIMEOUT;
			return m_iErrorState;
		}
	} while( !bExit );
	
#endif

#if defined(__LINUX__)					
	ssize_t bytesRead = 0;
	struct timeval clTimeout;
	fd_set fdReadSet;
	clTimeout.tv_sec=0;     // seconds to wait
	clTimeout.tv_usec=m_uiTimeOut * 1000;
	m_clTimer.start();
	
	do
	{	
		FD_ZERO( &fdReadSet );
		FD_SET( m_hDevice, &fdReadSet );

		iRetVal=select( FD_SETSIZE, &fdReadSet, NULL, NULL, &clTimeout );
		if(iRetVal < 0)
		{
			warning("Receive Error in select");
			return ERRID_IO_READERROR;
		}
		else if (iRetVal>0 && FD_ISSET(m_hDevice,&fdReadSet))
		{
			bytesRead = read( m_hDevice, &ucChar, 1 );
			if( bytesRead < 0 )
			{	warning( "Receive Error. Read error.\n" );
				m_iErrorState = ERRID_DEV_READERROR;
				return m_iErrorState;
			}
			else if( bytesRead == 1 )
			{	
				aucReadBuffer[uiReadBufferIndex++] = ucChar;
				if( ucChar == ETX )
					bExit = true;
			}
			else if( bytesRead == 0 )
			{
				warning( "Receive Error. Timeout\n");
				m_iErrorState = ERRID_DEV_READTIMEOUT;
				return m_iErrorState;
			}
		}
		m_clTimer.stop();
		fTime = m_clTimer.executionTime() * 1000;
		if( fTime > m_uiTimeOut * m_iModuleCount)
		{	bExit = true;
			warning( "Receive Error. Timeout. %d bytes received.\n", uiReadBufferIndex );
			m_iErrorState = ERRID_DEV_READTIMEOUT;
			return m_iErrorState;
		}
	} while( !bExit );

#endif

	for( i = 0; i < uiReadBufferIndex; i++ )
	{	
		if( aucReadBuffer[i] == STX )
		{	
			uiMessageBufferIndex = 0;
			bMessageComplete = false;
		}
		if( aucReadBuffer[i] == ETX )
			bMessageComplete = true;
		if( uiMessageBufferIndex > 22 )
		{	
			uiMessageBufferIndex = 0;
			bMessageComplete = false;
			warning( "More than 22 bytes!" );
		}
		aucMessageBuffer[uiMessageBufferIndex] = aucReadBuffer[i];
		uiMessageBufferIndex++;
	}

	if( bMessageComplete )
	{	// save net m_aucMessageData in aucDecodeBuffer[]
		if( (aucMessageBuffer[0] == STX) && (aucMessageBuffer[uiMessageBufferIndex-1] == ETX) )
		{	// Message frame begins with STX and ending with ETX
			for( i = 1; i < uiMessageBufferIndex-1; i++ )
			{ 
				if( aucMessageBuffer[i] == DLE )
				{	
					i++;
					aucDecodeBuffer[k] = aucMessageBuffer[i] - 0x80;
				}
				else if( aucMessageBuffer[i] == ETX || aucMessageBuffer[i] == STX )
				{	
					bDecodeError = true;
					break;
				}
				else
					aucDecodeBuffer[k] = aucMessageBuffer[i];
				k++;
			}
		}
		else
		{	// Anfang kein STX / Ende kein ETX
			warning( "Receive Error: STX/ETX framing incorrect.\n" );
			m_iErrorState = ERRID_DEV_READERROR;
			return m_iErrorState;
		}

		// Calculate net Len of Message 
		ucMessageLength = aucDecodeBuffer[1] & 0x0F;
		if( aucDecodeBuffer[0] & RSID_RECVDAT )
		{	// Message comes from Module
			if( !bDecodeError )
			{	
				if( ucMessageLength == k-3 )
				{	
					rclProtocolMessage.m_ucMessageLength = ucMessageLength;
					rclProtocolMessage.m_iModuleId = (((aucDecodeBuffer[0]&0x03)<<3)+((aucDecodeBuffer[1]&0xE0)>>5));
					for( i = 0; i < ucMessageLength; i++ )
						rclProtocolMessage.m_aucMessageData[i] = aucDecodeBuffer[i+2];
					rclProtocolMessage.m_uiMessageId = MSGID_ACK + rclProtocolMessage.m_iModuleId;
					m_iErrorState = 0;
					return m_iErrorState;
				}
				warning( "Receive Error: Length incorrect received %d instead of %d\n", k-3, ucMessageLength );
				m_iErrorState = ERRID_DEV_READERROR;
				return m_iErrorState;
			}
			// Decoder-Fehler: mittendrin STX oder ETX				
			warning( "Receive Error: STX/ETX inside message.\n" );
			m_iErrorState = ERRID_DEV_READERROR;
			return m_iErrorState;
		}
		// Message kommt nicht vom Modul (TelId != TELID_RECVDAT)
		warning( "Receive Error: MessageId incorrect.\n" );
		m_iErrorState = ERRID_DEV_READERROR;
		return m_iErrorState;
	}
	// Kein ETX empfangen...
	warning( "Receive Error: ETX not received.\n" );
	m_iErrorState = ERRID_DEV_READERROR;
	return m_iErrorState;
}

int CRS232Device::writeDevice(CProtocolMessage& rclProtocolMessage)
{
	int i = 0;
	unsigned int uiWriteBufferIndex = 0;
	unsigned long uiWriteLength = 0;
	unsigned char aucWriteBuffer[24];
	unsigned short uiSum = 0;
	unsigned char aucEncodeBuffer[11] = {0,0,0,0,0,0,0,0,0,0,0};
	int iRetVal;
	CRS232Message clRS232Message;

	m_iErrorState = 0;

	clRS232Message.m_aucMessageId[0] = (unsigned char)rclProtocolMessage.m_iModuleId >> 3;
	clRS232Message.m_aucMessageId[0] |= RSID_SENDDAT;
	clRS232Message.m_aucMessageId[1] = (unsigned char)rclProtocolMessage.m_iModuleId << 5;
	clRS232Message.m_aucMessageId[1] |= (unsigned char)rclProtocolMessage.m_ucMessageLength;
	clRS232Message.m_ucMessageLength = (unsigned char)rclProtocolMessage.m_ucMessageLength;
	memcpy(clRS232Message.m_aucMessageData, rclProtocolMessage.m_aucMessageData, rclProtocolMessage.m_ucMessageLength);

	aucEncodeBuffer[0] = clRS232Message.m_aucMessageId[0];
	aucEncodeBuffer[1] = clRS232Message.m_aucMessageId[1];
	for( i = 0; i < clRS232Message.m_ucMessageLength; i++ )
		aucEncodeBuffer[i+2] = clRS232Message.m_aucMessageData[i];
	for( i = 0; i < clRS232Message.m_ucMessageLength+2; i++ )
		uiSum += aucEncodeBuffer[i];
	aucEncodeBuffer[clRS232Message.m_ucMessageLength+2] = uiSum + (uiSum>>8);

	aucWriteBuffer[0] = STX;
	uiWriteBufferIndex = 1;

	for( i = 0; i < clRS232Message.m_ucMessageLength+3; i++ )
	{ 
		if( aucEncodeBuffer[i] == DLE || aucEncodeBuffer[i] == STX || aucEncodeBuffer[i] == ETX )
		{	
			aucWriteBuffer[uiWriteBufferIndex] = DLE;
			uiWriteBufferIndex++;
			aucWriteBuffer[uiWriteBufferIndex] = aucEncodeBuffer[i] + 0x80;
		}
		else
			aucWriteBuffer[uiWriteBufferIndex] = aucEncodeBuffer[i];
		uiWriteBufferIndex++;
	}

	aucWriteBuffer[uiWriteBufferIndex] = ETX;

#if defined(_WIN32)					
	DWORD iErrorCode;

	PurgeComm( m_hDevice, PURGE_TXABORT );
	PurgeComm( m_hDevice, PURGE_RXABORT );
	PurgeComm( m_hDevice, PURGE_TXCLEAR );
	PurgeComm( m_hDevice, PURGE_RXCLEAR );

	iRetVal = WriteFile( m_hDevice, aucWriteBuffer, uiWriteBufferIndex+1, &uiWriteLength, NULL );
	if (iRetVal == 0)
	{	
		// Transmission error
		void* lpMsgBuf;
		FormatMessage( 
			FORMAT_MESSAGE_ALLOCATE_BUFFER | 
			FORMAT_MESSAGE_FROM_SYSTEM | 
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			GetLastError(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR) &lpMsgBuf,
			0,
			NULL 
			);
		debug(1,"COMERROR:%s\n",lpMsgBuf);
		warning( "Transmission Error. Sent %ld bytes instead of &ld: \n", uiWriteLength, uiWriteBufferIndex+1 );
		ClearCommError( m_hDevice, &iErrorCode, NULL );
		m_iErrorState = ERRID_DEV_WRITEERROR;
		return m_iErrorState;
	}

	if( uiWriteLength != uiWriteBufferIndex+1 )
	{	warning( "Transmission Error. Sent %ld bytes instead of &ld: \n", uiWriteLength, uiWriteBufferIndex+1 );
		m_iErrorState = ERRID_DEV_WRITEERROR;
		return m_iErrorState;
	}
#endif

#if defined(__QNX__)					
	tcflush( m_hDevice, TCIOFLUSH );
	
	uiWriteLength = write(m_hDevice, aucWriteBuffer, uiWriteBufferIndex+1);
	if( uiWriteLength != uiWriteBufferIndex+1 )
	{	warning( "Transmission Error %d. Sent %ld bytes instead of %ld.\n", errno, uiWriteLength, uiWriteBufferIndex+1 );
		m_iErrorState = ERRID_DEV_WRITEERROR;
		return m_iErrorState;
	}
	tcdrain( m_hDevice );
#endif

#if defined(__LINUX__)					
	tcflush( m_hDevice, TCIOFLUSH );
	
	uiWriteLength = write(m_hDevice, aucWriteBuffer, uiWriteBufferIndex+1);
	if( uiWriteLength != uiWriteBufferIndex+1 )
	{	warning( "Transmission Error. Sent %ld bytes instead of %ld.\n", uiWriteLength, uiWriteBufferIndex+1 );
		m_iErrorState = ERRID_DEV_WRITEERROR;
		return m_iErrorState;
	}
	tcdrain( m_hDevice );
#endif

	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CRS232Device::CRS232Device() : m_hDevice(0), m_iDeviceId(-1), m_uiBaudRate(0), m_uiTimeOut(30), m_clTimer(util_REAL_TIME)
{
	initMessage("CRS232Device", g_iDebugLevel, g_bDebug, g_bDebugFile);
}

CRS232Device::CRS232Device(const CRS232Device& rclRS232Device)
{
	error(-1, "Sorry constructor is not implemented");
}

CRS232Device::~CRS232Device()
{
	exit();
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CRS232Device& CRS232Device::operator=(const CRS232Device& rclRS232Device)
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

void CRS232Device::setTimeOut(unsigned long uiTimeOut)
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

int CRS232Device::init()
{
	return init(m_acInitString);
}

int CRS232Device::init(const char* acInitString)
{
	InitializeCriticalSection(&m_csDevice);
	char* pcToken;
	char acString[128];
	char acDevice[128];
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
	if( strcmp( pcToken, "RS232" ) != 0 )
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

#if defined(_WIN32)					
	COMMTIMEOUTS commtimeouts;
	DCB dcb;
	sprintf( acDevice, "COM%d", m_iDeviceId );
	m_hDevice = CreateFile( acDevice, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL );
	if( m_hDevice == INVALID_HANDLE_VALUE )
	{	warning( "Could not initialize %s\n", acDevice );
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	if( !GetCommState( m_hDevice, &dcb ) )
	{	warning( "GetCommState-Error %d\n", GetLastError() );
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	switch( m_iBaudRate )
	{
	case 1200:
		dcb.BaudRate = CBR_1200;	
		break;
	case 2400:
		dcb.BaudRate = CBR_2400;	
		break;
	case 4800:
		dcb.BaudRate = CBR_4800;	
		break;
	case 9600:
		dcb.BaudRate = CBR_9600;	
		break;
	case 19200:
		dcb.BaudRate = CBR_19200;	
		break;
	case 38400:
		dcb.BaudRate = CBR_38400;	
		break;
	case 57600:
		dcb.BaudRate = CBR_57600;	
		break;
	case 115200:
		dcb.BaudRate = CBR_115200;	
		break;
	default:
		dcb.BaudRate = CBR_9600;	
		break;
	}

	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;
	dcb.fRtsControl = RTS_CONTROL_DISABLE; //RTS_CONTROL_TOGGLE; //RTS_CONTROL_HANDSHAKE; //RTS_CONTROL_ENABLE;
	dcb.fDtrControl = DTR_CONTROL_HANDSHAKE; //DTR_CONTROL_HANDSHAKE; //DTR_CONTROL_ENABLE;
	
	if( !SetCommState( m_hDevice, &dcb) )
	{	warning( "SetCommState-Error %d\n", GetLastError() );
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}
	commtimeouts.ReadIntervalTimeout = 0;
	commtimeouts.ReadTotalTimeoutMultiplier = 0; 
	commtimeouts.ReadTotalTimeoutConstant = m_uiTimeOut;  ///Timeout for Read ms 
	commtimeouts.WriteTotalTimeoutMultiplier = 0; 
	commtimeouts.WriteTotalTimeoutConstant = m_uiTimeOut;  ///Timeout for Write ms 
	SetCommTimeouts(m_hDevice,&commtimeouts);

#endif

#if defined(__QNX__)					
	sprintf(acDevice,"/dev/ser%d",m_iDeviceId);
	m_hDevice = open(acDevice, O_RDWR|O_NONBLOCK );
	if( m_hDevice <=0 )   
	{	warning( "Could not initialize %s\n", acDevice );
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	struct termios clPortOptions;
	tcgetattr(m_hDevice, &clPortOptions);		//get current port-options

	switch( m_iBaudRate )
	{
	case 1200:
		cfsetispeed(&clPortOptions, B1200);
		cfsetospeed(&clPortOptions, B1200);
		break;
	case 2400:
		cfsetispeed(&clPortOptions, B2400);
		cfsetospeed(&clPortOptions, B2400);
		break;
	case 4800:
		cfsetispeed(&clPortOptions, B4800);
		cfsetospeed(&clPortOptions, B4800);
		break;
	case 9600:
		cfsetispeed(&clPortOptions, B9600);
		cfsetospeed(&clPortOptions, B9600);
		break;
	case 19200:
		cfsetispeed(&clPortOptions, B19200);
		cfsetospeed(&clPortOptions, B19200);
		break;
	case 38400:
		cfsetispeed(&clPortOptions, B38400);
		cfsetospeed(&clPortOptions, B38400);
		break;
	case 57600:
		cfsetispeed(&clPortOptions, B57600);
		cfsetospeed(&clPortOptions, B57600);
		break;
	case 115200:
		cfsetispeed(&clPortOptions, B115200);
		cfsetospeed(&clPortOptions, B115200);
		break;
	default:
		cfsetispeed(&clPortOptions, B9600);
		cfsetospeed(&clPortOptions, B9600);
		break;
	}

	clPortOptions.c_cflag |= CREAD|CS8;
	clPortOptions.c_lflag = 0;
	if( tcsetattr(m_hDevice, TCSANOW, &clPortOptions) != 0 )
	{	warning( "open: Could not set attributes\n" );
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}	
	
#endif

#if defined(__LINUX__)					
	sprintf(acDevice,"/dev/ttyS%d",m_iDeviceId-1);
	m_hDevice = open( acDevice, O_RDWR );	//open Port
	if( m_hDevice == -1 )
	{	warning( "open: Could not initialize %s\n", acDevice );
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}

	struct termios clPortOptions;
	tcgetattr(m_hDevice, &clPortOptions);		//get current port-options

	switch( m_iBaudRate )
	{
	case 1200:
		cfsetispeed(&clPortOptions, B1200);
		cfsetospeed(&clPortOptions, B1200);
		break;
	case 2400:
		cfsetispeed(&clPortOptions, B2400);
		cfsetospeed(&clPortOptions, B2400);
		break;
	case 4800:
		cfsetispeed(&clPortOptions, B4800);
		cfsetospeed(&clPortOptions, B4800);
		break;
	case 9600:
		cfsetispeed(&clPortOptions, B9600);
		cfsetospeed(&clPortOptions, B9600);
		break;
	case 19200:
		cfsetispeed(&clPortOptions, B19200);
		cfsetospeed(&clPortOptions, B19200);
		break;
	case 38400:
		cfsetispeed(&clPortOptions, B38400);
		cfsetospeed(&clPortOptions, B38400);
		break;
	case 57600:
		cfsetispeed(&clPortOptions, B57600);
		cfsetospeed(&clPortOptions, B57600);
		break;
	case 115200:
		cfsetispeed(&clPortOptions, B115200);
		cfsetospeed(&clPortOptions, B115200);
		break;
	default:
		cfsetispeed(&clPortOptions, B9600);
		cfsetospeed(&clPortOptions, B9600);
		break;
	}

	clPortOptions.c_iflag = 0;
	clPortOptions.c_oflag = 0;
	clPortOptions.c_cflag |= CLOCAL|CREAD|CS8|CSIZE;
	clPortOptions.c_lflag = 0;
	if( tcsetattr(m_hDevice, TCSANOW, &clPortOptions) != 0 )
	{	warning( "open: Could not set attributes\n" );
		m_iErrorState = ERRID_DEV_INITERROR;
		return m_iErrorState;
	}	
#endif

	m_iErrorState = clearReadQueue();
	if(m_iErrorState != 0)
		return m_iErrorState;

	if(m_iErrorState == 0)
		m_bInitFlag = true;
	updateModuleIdMap();
	return m_iErrorState;
}

int CRS232Device::exit()
{
	m_iErrorState = 0;
	if(!m_bInitFlag)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	EnterCriticalSection(&m_csDevice);
#if defined(_WIN32)					
	if( !CloseHandle( m_hDevice ) )
	{	
		warning( "Error closing Device.\n" );
		m_iErrorState = ERRID_DEV_EXITERROR;
		return m_iErrorState;
	}
#endif

#if defined(__QNX__)					
	if( close(m_hDevice) < 0 )
	{	warning( "Error closing Device.\n" );
		m_iErrorState = ERRID_DEV_EXITERROR;
		return m_iErrorState;
	}	
#endif

#if defined(__LINUX__)					
	if( close(m_hDevice) < 0 )
	{	warning( "Error closing Device.\n" );
		m_iErrorState = ERRID_DEV_EXITERROR;
		return m_iErrorState;
	}
#endif

	m_bInitFlag = false;
	LeaveCriticalSection(&m_csDevice);
	DeleteCriticalSection(&m_csDevice);
	return m_iErrorState;
}
