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

