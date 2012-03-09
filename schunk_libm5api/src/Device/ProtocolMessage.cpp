
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


#include "ProtocolMessage.h"
#include <string.h>

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CProtocolMessage::CProtocolMessage() : 
						m_uiMessageId(0), 
						m_ucMessageLength(0), 
						m_ucMessageState(0), 
						m_bRTRFlag(false), 
						m_fTime(0.0),
						m_iModuleId(0)
{
}

CProtocolMessage::CProtocolMessage(const CProtocolMessage& rclProtocolMessage) : 
						m_uiMessageId(rclProtocolMessage.m_uiMessageId), 
						m_ucMessageLength(rclProtocolMessage.m_ucMessageLength), 
						m_ucMessageState(rclProtocolMessage.m_ucMessageState), 
						m_bRTRFlag(rclProtocolMessage.m_bRTRFlag), 
						m_fTime(rclProtocolMessage.m_fTime),
						m_iModuleId(rclProtocolMessage.m_iModuleId)
{
	memcpy(m_aucMessageData, rclProtocolMessage.m_aucMessageData, m_ucMessageLength);
}

CProtocolMessage::~CProtocolMessage()
{
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CProtocolMessage& CProtocolMessage::operator=(const CProtocolMessage& rclProtocolMessage)
{
	m_uiMessageId = rclProtocolMessage.m_uiMessageId;
	m_ucMessageLength = rclProtocolMessage.m_ucMessageLength;
	m_ucMessageState = rclProtocolMessage.m_ucMessageState;
	m_bRTRFlag = rclProtocolMessage.m_bRTRFlag;
	m_fTime = rclProtocolMessage.m_fTime;
	m_iModuleId = rclProtocolMessage.m_iModuleId;
	memcpy(m_aucMessageData, rclProtocolMessage.m_aucMessageData, m_ucMessageLength);
	return *this;
}

