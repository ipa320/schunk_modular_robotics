
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


#include "CP5X11Device.h"
#define DPN_WIN
#include "../include/dpn_user.h"
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
int CCP5X11Device::getDeviceError(int iErrorState)
{
	return iErrorState;
}

int CCP5X11Device::setBaudRate()
{	
	m_iErrorState = 0;
	return m_iErrorState;
}

int CCP5X11Device::setMessageId(unsigned long uiMessageId)
{
	m_iErrorState = 0;
	return m_iErrorState;
}

int CCP5X11Device::clearReadQueue()
{	
	m_iErrorState = 0;
	return m_iErrorState;
}

int CCP5X11Device::reinit(unsigned char ucBaudRateId)
{
	m_iErrorState = 0;
	return m_iErrorState;
}

int CCP5X11Device::readDevice(CProtocolMessage& rclProtocolMessage)
{	
	unsigned short int iRetVal = 0;
	double fTime = 0;
	m_iErrorState = 0;

/*	dpn_interface dpn;
	dpn.reference.board_select = m_hDevice;
	dpn.reference.access = m_access;
	dpn.stat_nr = rclProtocolMessage.m_iModuleId;
	dpn.length = 255;
	dpn_interface* pDpn = &dpn;
*/
	struct dpn_interface_m  dpnIn;  
	dpnIn.dpn_if_single[0].reference.access = m_access;
	dpnIn.dpn_if_single[0].reference.board_select = m_hDevice;
	dpnIn.dpn_if_single[0].stat_nr = rclProtocolMessage.m_iModuleId;
	for( int j = 1; j < DPN_MULTIPLE_SIZE; j++ )
		dpnIn.dpn_if_single[j].stat_nr = DPN_IF_S_UNUSED;
	dpn_interface_m* pDpnIn = &dpnIn;
	
	m_clTimer.start();
	do
	{	
		Sleep(1);
		//iRetVal = dpn_in_slv( pDpn );
		iRetVal = dpn_in_slv_m( &dpnIn );

/*		for( int i = 0; i < 16; i++ )
			printf( "%02x ", pDpnIn->dpn_if_single[0].user_data[i] );
		printf( "\n" );
*/
		m_clTimer.stop();
		fTime = m_clTimer.executionTime() * 1000;
		if(fTime > m_uiTimeOut)
		{	
			warning("CP5X11 readDevice timeout");
			m_iErrorState = ERRID_DEV_READERROR;
			return m_iErrorState;
		}

		if( iRetVal != DPN_NO_ERROR )
		{	warning("CP5X11 dpn_in_slv failure: %x", iRetVal );
			m_iErrorState = ERRID_DEV_READERROR;
			return m_iErrorState;
		}

/*		if( rclProtocolMessage.m_ucMessageLength == 1 )
			if( pDpn->user_data[0] == rclProtocolMessage.m_aucMessageData[0] )
				break;
		if( rclProtocolMessage.m_ucMessageLength > 1 )
			if( pDpn->user_data[0] == rclProtocolMessage.m_aucMessageData[0] && 
					pDpn->user_data[1] == rclProtocolMessage.m_aucMessageData[1] )
				break;
*/
		if( rclProtocolMessage.m_ucMessageLength == 1 )
			if( pDpnIn->dpn_if_single[0].user_data[0] == rclProtocolMessage.m_aucMessageData[0] )
				break;
		if( rclProtocolMessage.m_ucMessageLength > 1 )
			if( pDpnIn->dpn_if_single[0].user_data[0] == rclProtocolMessage.m_aucMessageData[0] && 
					pDpnIn->dpn_if_single[0].user_data[1] == rclProtocolMessage.m_aucMessageData[1] )
				break;
/*			
		for( i = 0; i < 8; i++ )
			printf( "%02x ", rclProtocolMessage.m_aucMessageData[i] );
		printf( "\n\n" );
		if( kbhit() )
			break;
*/
	}	while( 1 );

/*	if( pDpn->length != 16 )
	{	warning( "CP5X11: Slave does not have 16 input bytes" );
		m_iErrorState = ERRID_DEV_READERROR;
		return m_iErrorState;
	}

	if( pDpn->sys_state != DPN_SYS_OPERATE )
	{	warning( "CP5X11: Master not in in operate mode" );
		m_iErrorState = ERRID_DEV_READERROR;
		return m_iErrorState;
	}
*/
	rclProtocolMessage.m_uiMessageId = MSGID_ACK + rclProtocolMessage.m_iModuleId;
	memcpy( rclProtocolMessage.m_aucMessageData, pDpnIn->dpn_if_single[0].user_data, 8 );
	return m_iErrorState;
}

int CCP5X11Device::writeDevice(CProtocolMessage& rclProtocolMessage)
{	
	unsigned short int iRetVal = 0;
	m_iErrorState = 0;

	if( rclProtocolMessage.m_ucMessageLength < 8 )
		rclProtocolMessage.m_aucMessageData[7] = m_teaser++;

/*	dpn_interface dpn;
	dpn.reference.board_select = m_hDevice;
	dpn.reference.access = m_access;
	dpn.stat_nr = rclProtocolMessage.m_iModuleId;
	dpn.length = 8;
	for( int i = 0; i < dpn.length; i++ )
		dpn.user_data[i] = rclProtocolMessage.m_aucMessageData[i];
	dpn_interface* pDpn = &dpn;
*/
	struct dpn_interface_m  dpnOut;  
	dpnOut.dpn_if_single[0].reference.access = m_access;
	dpnOut.dpn_if_single[0].reference.board_select = m_hDevice;
	dpnOut.dpn_if_single[0].stat_nr = rclProtocolMessage.m_iModuleId;
	dpnOut.dpn_if_single[0].length = 8;
	for( int i = 0; i < 8; i++ )
		dpnOut.dpn_if_single[0].user_data[i] = rclProtocolMessage.m_aucMessageData[i];
	for( int j = 1; j < DPN_MULTIPLE_SIZE; j++ )
		dpnOut.dpn_if_single[j].stat_nr = DPN_IF_S_UNUSED;
	dpn_interface_m* pDpnOut = &dpnOut;

	iRetVal = dpn_out_slv_m( &dpnOut );

/*	iRetVal = dpn_out_slv( pDpn );
	if( iRetVal != DPN_NO_ERROR )
	{	warning( "CP5X11 dpn_out_slv failure: %x", iRetVal );
		m_iErrorState = ERRID_DEV_WRITEERROR;
	}

	if( pDpn->sys_state != DPN_SYS_OPERATE )
	{	warning( "CP5X11: Master not in in operate mode" );
		m_iErrorState = ERRID_DEV_WRITEERROR;
	}
*/
	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CCP5X11Device::CCP5X11Device() : m_hDevice(NULL), m_uiTimeOut(50), m_clTimer(util_REAL_TIME)
{
	initMessage("CCP5X11Device", g_iDebugLevel, g_bDebug, g_bDebugFile);
}

CCP5X11Device::CCP5X11Device(const CCP5X11Device& rclCP5X11Device)
{
	error(-1, "Sorry constructor is not implemented");
}

CCP5X11Device::~CCP5X11Device()
{
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CCP5X11Device& CCP5X11Device::operator=(const CCP5X11Device& rclCP5X11Device)
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

void CCP5X11Device::setTimeOut(unsigned long uiTimeOut)
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

int CCP5X11Device::init()
{
	return init(m_acInitString);
}

int CCP5X11Device::init(const char* acInitString)
{
	InitializeCriticalSection(&m_csDevice);

	unsigned short int iRetVal = 0;
	char* pcToken;
	char acString[128];
	m_bInitFlag = false;
	m_hDevice = NULL;
	m_iErrorState = 0;
	dpn_interface dpn;
	dpn_interface dpnReinit;

	strncpy( m_acInitString, acInitString, 128 );
	strncpy( acString, acInitString, 128 );

	pcToken = strtok( acString, ":" );
	if( !pcToken )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	if( strcmp( pcToken, "CP5X11" ) != 0 )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}

	pcToken = strtok( NULL, "," );
	if( !pcToken )
	{	m_iErrorState = ERRID_DEV_BADINITSTRING;
		return m_iErrorState;
	}
	m_hDevice = atoi( pcToken );
	
	try
	{
		// First init to retrieve infromation what slaves are configured
		dpn.reference.board_select = m_hDevice;
		dpn.reference.access = DPN_SYS_NOT_CENTRAL | DPN_ROLE_NOT_CENTRAL;
		dpn.length = 126;
		for( int i = 0; i <	126; i++ )
			dpn.user_data[i] = DPN_SLV_NO_ACCESS; //DPN_SLV_WRITE_READ; 
		dpn_interface* pDpn = &(dpn);

		iRetVal = dpn_init( pDpn );
		if( iRetVal != DPN_NO_ERROR )
		{	warning( "CP5X11 dpn_init failed: %x", iRetVal );
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}
		m_hDevice = pDpn->reference.board_select;
		m_access = pDpn->reference.access;

		// Get board config
		dpn.reference.board_select = m_hDevice;
		dpn.length = 126;
		pDpn = &(dpn);
		iRetVal = dpn_read_cfg( pDpn );
		if( iRetVal != DPN_NO_ERROR )
		{	warning( "CP5X11 dpn_init failed: %x", iRetVal );
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}

		// save configuration
		for( i = 0; i < pDpn->length; i++ )
		{	if( pDpn->user_data[i] == DPN_CFG_NORM )
				dpnReinit.user_data[i] = DPN_SLV_WRITE_READ;
			else
				dpnReinit.user_data[i] = DPN_SLV_NO_ACCESS; 
		}
		dpn.reference.board_select = m_hDevice;
		dpn.reference.access = m_access;
		pDpn = &(dpn);

		// reset to close the board
		iRetVal = dpn_reset( pDpn );
		if( iRetVal != DPN_NO_ERROR )
		{	warning( "dpn_reset failed with %x", iRetVal );
			m_iErrorState = ERRID_DEV_EXITERROR;
		}
		
		// Reinit after knowing what slaves are configured...
		dpnReinit.reference.board_select = m_hDevice;
		dpnReinit.reference.access = DPN_SYS_NOT_CENTRAL | DPN_ROLE_NOT_CENTRAL;
		dpnReinit.length = 126;
		pDpn = &(dpnReinit);

		iRetVal = dpn_init( pDpn );
		if( iRetVal != DPN_NO_ERROR )
		{	warning( "CP5X11 dpn_init failed: %x", iRetVal );
			m_iErrorState = ERRID_DEV_INITERROR;
			return m_iErrorState;
		}

		m_hDevice = pDpn->reference.board_select;
		m_access = pDpn->reference.access;
	}
	catch(...)
	{
			warning("init CP5X11 device failed no library found");
			m_iErrorState = ERRID_DEV_NOLIBRARY;
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

int CCP5X11Device::exit()
{	
	unsigned short int iRetVal = 0;
	m_iErrorState = 0;
	dpn_interface dpn;

	if(!m_bInitFlag)
	{	
		warning("device not initialzed");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
	}
	EnterCriticalSection(&m_csDevice);
	dpn.reference.board_select = m_hDevice;
	dpn.reference.access = m_access;
	dpn_interface* pDpn = &(dpn);

	iRetVal = dpn_reset( pDpn );
	if( iRetVal != DPN_NO_ERROR )
	{	
		warning( "Could not exit CP5X11" );
		m_iErrorState = ERRID_DEV_EXITERROR;
	}

	m_hDevice = NULL;
	m_bInitFlag = false;
	LeaveCriticalSection(&m_csDevice);
	DeleteCriticalSection(&m_csDevice);
	return m_iErrorState;
}
