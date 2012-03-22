
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


#include "Device.h"
#include "RS232Device.h"
#ifdef USE_ESD
#include "ESDDevice.h"
#endif
#ifdef USE_PCAN
#include "PCanDevice.h"
#endif
#if defined (_WIN32)
	#include "CP5X11Device.h"
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

void CDevice::charStateToLongState(unsigned char ucShort, unsigned long* puiState)
{
	*puiState = 0;
	if(ucShort & 1)
		*puiState |= STATEID_MOD_ERROR;
	if(ucShort & 2)
		*puiState |= STATEID_MOD_SWR;
	if(ucShort & 4)
		*puiState |= STATEID_MOD_SW1;
	if(ucShort & 8)
		*puiState |= STATEID_MOD_SW2;
	if(ucShort & 16)
		*puiState |= STATEID_MOD_MOTION;
	if(ucShort & 32)
		*puiState |= STATEID_MOD_RAMP_END;
	if(ucShort & 64)
		*puiState |= STATEID_MOD_INPROGRESS;
	if(ucShort & 128)
		*puiState |= STATEID_MOD_FULLBUFFER;
}

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CDevice::CDevice() : m_bInitFlag(false), 
					m_iBaudRate(250), 
					m_iModuleCount(0), 
					m_iModuleCountMax(31), 
					m_clTimer(util_REAL_TIME),
					m_iErrorState(0)		
{ 
	m_acName[0] = '\0';
	m_acInitString[0] = '\0';
	strncpy(m_acRevision,"$Revision: 1.45 $",20);
}

CDevice::CDevice(const CDevice& rclDevice)
{
	error(-1, "Sorry constructor is not implemented");
}

CDevice::~CDevice()
{
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CDevice& CDevice::operator=(const CDevice& rclDevice)
{
	error(-1, "Sorry operator= is not implemented");
	return *this;
}

// ========================================================================== ;
//                                                                            ;
// ---- query functions ----------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

bool CDevice::getInitFlag()
{
	return m_bInitFlag;
}

const char* CDevice::getRevision()
{
	return m_acRevision;
}

const char* CDevice::getName()
{
	return m_acName;
}

const char* CDevice::getInitString()
{
	return m_acInitString;
}

int CDevice::getBaudRate()
{
	return m_iBaudRate;
}

int CDevice::getModuleCount()
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	return m_iModuleCount;
}

int CDevice::getModuleIdMap(std::vector<int>& raiModuleId)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	raiModuleId.resize(m_iModuleCount);
	for(int i=0; i < m_iModuleCount; i++)
		raiModuleId[i] = m_aiModuleId[i];

	return m_iModuleCount;
}

int CDevice::getModuleState(int iModuleId, unsigned long* puiState)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_ACT_STATE, puiState);

	return m_iErrorState;
}

int CDevice::getModuleType(int iModuleId, unsigned char* pucValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	unsigned long uiConfig = 0;
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_DEF_CONFIG, &uiConfig);
	if(m_iErrorState == 0)
	{
		if (uiConfig & CONFIGID_MOD_LINEAR)
			*pucValue = TYPEID_MOD_LINEAR;
		else
			*pucValue = TYPEID_MOD_ROTARY;
	}
	return m_iErrorState;
}

int CDevice::getModuleVersion(int iModuleId, unsigned short* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedShort(iModuleId, CMDID_GETPARAM, PARID_DEF_VERSION, puiValue);

	return m_iErrorState;
}

int CDevice::getModuleSerialNo(int iModuleId, unsigned long* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_DEF_SERIALNO, puiValue);
        debug(0,"error state: 0x%04x",m_iErrorState);

	return m_iErrorState;
}

int CDevice::getDefConfig(int iModuleId, unsigned long* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_DEF_CONFIG, puiValue);

	return m_iErrorState;
}

int CDevice::getDefSetup(int iModuleId, unsigned long* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x3500)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_DEF_SETUP, puiValue);

	return m_iErrorState;
}

int CDevice::getDefBaudRate(int iModuleId, unsigned char* pucValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedChar(iModuleId, CMDID_GETPARAM, PARID_DEF_CANBAUDRATE, pucValue);

	return m_iErrorState;
}

int CDevice::getDefCANBaudRate(int iModuleId, unsigned char* pucValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedChar(iModuleId, CMDID_GETPARAM, PARID_DEF_CANBAUDRATE, pucValue);

	return m_iErrorState;
}

int CDevice::getDefRSBaudRate(int iModuleId, unsigned char* pucValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedChar(iModuleId, CMDID_GETPARAM, PARID_DEF_RSBAUDRATE, pucValue);

	return m_iErrorState;
}

int CDevice::getDefBurnCount(int iModuleId, unsigned char* pucValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedChar(iModuleId, CMDID_GETPARAM, PARID_DEF_BURNCOUNT, pucValue);

	return m_iErrorState;
}

int CDevice::getDefGearRatio(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FGEARRATIO, pfValue);

	return m_iErrorState;
}

int CDevice::getDefLinearRatio(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FLINEARRATIO, pfValue);

	return m_iErrorState;
}

int CDevice::getDefCurOffset(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FCUROFFSET, pfValue);

	return m_iErrorState;
}

int CDevice::getDefCurRatio(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FCURRATIO, pfValue);

	return m_iErrorState;
}

int CDevice::getDefBrakeTimeOut(int iModuleId, unsigned short* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedShort(iModuleId, CMDID_GETPARAM, PARID_DEF_BRAKETIMEOUT, puiValue);

	return m_iErrorState;
}

int CDevice::getDefIncPerTurn(int iModuleId, unsigned long* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_DEF_INCPERTURN, puiValue);

	return m_iErrorState;
}

int CDevice::getDefDioData(int iModuleId, unsigned long* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
//	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_DEF_DIODATA, puiValue);

	*puiValue = 0;
	return m_iErrorState;
}

int CDevice::getDefA0(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_DEF_A0, piValue);

	return m_iErrorState;
}

int CDevice::getDefC0(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_DEF_C0, piValue);

	return m_iErrorState;
}

int CDevice::getDefDamp(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_DEF_DAMP, piValue);

	return m_iErrorState;
}

int CDevice::getDefHomeOffset(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FHOMEOFFSET, pfValue);

	return m_iErrorState;
}

int CDevice::getDefHomeVel(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FHOMEVEL, pfValue);

	return m_iErrorState;
}

int CDevice::getDefMinPos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FMINPOS, pfValue);

	return m_iErrorState;
}

int CDevice::getDefMaxPos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FMAXPOS, pfValue);

	return m_iErrorState;
}

int CDevice::getDefMaxVel(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FMAXVEL, pfValue);

	return m_iErrorState;
}

int CDevice::getDefMaxAcc(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FMAXACC, pfValue);

	return m_iErrorState;
}

int CDevice::getDefMaxCur(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FMAXCUR, pfValue);

	return m_iErrorState;
}

int CDevice::getDefMaxDeltaPos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_DEF_FMAXDELTAPOS, pfValue);

	return m_iErrorState;
}

int CDevice::getConfig(int iModuleId, unsigned long* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_ACT_CONFIG, puiValue);

	return m_iErrorState;
}

int CDevice::getHomeOffset(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FHOMEOFFSET, pfValue);

	return m_iErrorState;
}

int CDevice::getHomeOffsetInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IHOMEOFFSET, piValue);

	return m_iErrorState;
}

int CDevice::getIncRatio(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FINCRATIO, pfValue);

	return m_iErrorState;
}

int CDevice::getDioData(int iModuleId, unsigned long* puiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readUnsignedLong(iModuleId, CMDID_GETPARAM, PARID_ACT_DIODATA, puiValue);

	return m_iErrorState;
}

int CDevice::getA0(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_A0, piValue);

	return m_iErrorState;
}

int CDevice::getC0(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_C0, piValue);

	return m_iErrorState;
}

int CDevice::getDamp(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_DAMP, piValue);

	return m_iErrorState;
}

int CDevice::getPos(int iModuleId, float* pfPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FPOS, pfPos);

	return m_iErrorState;
}

int CDevice::getVel(int iModuleId, float* pfVel)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FVEL, pfVel);

	return m_iErrorState;
}

int CDevice::getCur(int iModuleId, float* pfCur)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FCUR, pfCur);

	return m_iErrorState;
}

int CDevice::getMinPos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMINPOS, pfValue);

	return m_iErrorState;
}

int CDevice::getMaxPos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMAXPOS, pfValue);

	return m_iErrorState;
}


int CDevice::getMaxVel(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMAXVEL, pfValue);

	return m_iErrorState;
}

int CDevice::getMaxAcc(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMAXACC, pfValue);

	return m_iErrorState;
}

int CDevice::getMaxCur(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMAXCUR, pfValue);

	return m_iErrorState;
}

int CDevice::getDeltaPos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FDELTAPOS, pfValue);

	return m_iErrorState;
}

int CDevice::getMaxDeltaPos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMAXDELTAPOS, pfValue);

	return m_iErrorState;
}

int CDevice::getSavePos(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x2518 || (m_auiModuleVersion[i] >= 0x3500 && m_auiModuleVersion[i] < 0x3518))
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FSAVEPOS, pfValue);

	return m_iErrorState;
}

int CDevice::getIPolVel(int iModuleId, float* pfVel)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FIPOLVEL, pfVel);

	return m_iErrorState;
}

int CDevice::getPosCountInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IPOSCOUNT, piValue);

	return m_iErrorState;
}

int CDevice::getPosInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IPOS, piValue);

	return m_iErrorState;
}

int CDevice::getVelInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IVEL, piValue);

	return m_iErrorState;
}

int CDevice::getCurInc(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_ICUR, piValue);

	return m_iErrorState;
}

int CDevice::getMinPosInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IMINPOS, piValue);

	return m_iErrorState;
}

int CDevice::getMaxPosInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IMAXPOS, piValue);

	return m_iErrorState;
}

int CDevice::getMaxVelInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IMAXVEL, piValue);

	return m_iErrorState;
}

int CDevice::getMaxAccInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IMAXACC, piValue);

	return m_iErrorState;
}

int CDevice::getDeltaPosInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IDELTAPOS, piValue);

	return m_iErrorState;
}

int CDevice::getMaxDeltaPosInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IMAXDELTAPOS, piValue);

	return m_iErrorState;
}

int CDevice::getStateDioPos(int iModuleId, unsigned long* puiState, unsigned char* pucDio, float* pfPos)
{
	unsigned char ucState;
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x2518 || (m_auiModuleVersion[i] >= 0x3500 && m_auiModuleVersion[i] < 0x3518))
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloatUnsignedChars(iModuleId, CMDID_GETPARAM, PARID_ACT_FPOSSTATEDIO, pfPos, &ucState, pucDio);
	if(m_iErrorState == 0)
		charStateToLongState(ucState, puiState);
	return m_iErrorState;
}

int CDevice::getHomeVel(int iModuleId, float* pfVel)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x2518 || (m_auiModuleVersion[i] >= 0x3500 && m_auiModuleVersion[i] < 0x3518))
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FHOMEVEL, pfVel);

	return m_iErrorState;
}

int CDevice::getHomeVelInc(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x2518 || (m_auiModuleVersion[i] >= 0x3500 && m_auiModuleVersion[i] < 0x3518))
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_IHOMEVEL, piValue);

	return m_iErrorState;
}

int CDevice::getSyncTime(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x3602)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_SYNCTIME, piValue);

	return m_iErrorState;
}

int CDevice::getRawMotorCurrent(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x3602)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_RAWMOTCUR, piValue);

	return m_iErrorState;
}

int CDevice::getRawMotorSupply(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x3602)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_RAWMOTSUPPLY, piValue);

	return m_iErrorState;
}

int CDevice::getRawTemperature(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x3602)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_RAWTEMP, piValue);

	return m_iErrorState;
}

int CDevice::getRawLogicSupply(int iModuleId, short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x3602)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_RAWLOGICSUPPLY, piValue);

	return m_iErrorState;
}

int CDevice::getLoadLimit(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x351A )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_LOADLIMIT, piValue);

	return m_iErrorState;
}

int CDevice::getMaxLoadGradient(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x351A )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_MAXLOADGRADIENT, piValue);

	return m_iErrorState;
}

int CDevice::getLoadDeltaTime(int iModuleId, unsigned short* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x351A)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readShort(iModuleId, CMDID_GETPARAM, PARID_ACT_LOADDELTATIME, (short*)piValue);

	return m_iErrorState;
}

int CDevice::getMotorCurrent(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMOTCUR, pfValue);

	return m_iErrorState;
}

int CDevice::getMotorSupply(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FMOTSUPPLY, pfValue);

	return m_iErrorState;
}

int CDevice::getTemperature(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FTEMP, pfValue);

	return m_iErrorState;
}

int CDevice::getLogicSupply(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_FLOGICSUPPLY, pfValue);

	return m_iErrorState;
}

int CDevice::getMinLogicVoltage(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_MINLOGIC, pfValue);

	return m_iErrorState;
}

int CDevice::getMaxLogicVoltage(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_MAXLOGIC, pfValue);

	return m_iErrorState;
}

int CDevice::getMinMotorVoltage(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_MINMOTOR, pfValue);

	return m_iErrorState;
}

int CDevice::getMaxMotorVoltage(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_MAXMOTOR, pfValue);

	return m_iErrorState;
}

int CDevice::getNominalMotorCurrent(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_NOMCUR, pfValue);

	return m_iErrorState;
}

int CDevice::getMaximumMotorCurrent(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_HMAXCUR, pfValue);

	return m_iErrorState;
}

int CDevice::getLogicUndershootTime(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_LOGICUNDERSHOOT, piValue);

	return m_iErrorState;
}

int CDevice::getLogicOvershootTime(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_LOGICOVERSHOOT, piValue);

	return m_iErrorState;
}

int CDevice::getMotorUndershootTime(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_MOTORUNDERSHOOT, piValue);

	return m_iErrorState;
}

int CDevice::getMotorOvershootTime(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_MOTOROVERSHOOT, piValue);

	return m_iErrorState;
}

int CDevice::getNomCurOvershootTime(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_NOMCUROVERSHOOT, piValue);

	return m_iErrorState;
}

int CDevice::getHMaxCurOvershootTime(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_HMAXCUROVERSHOOT, piValue);

	return m_iErrorState;
}

int CDevice::getKpPWMLimit(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_KP_PWMLIM, piValue);

	return m_iErrorState;
}

int CDevice::getCurrentLimit(int iModuleId, float* pfValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readFloat(iModuleId, CMDID_GETPARAM, PARID_ACT_CURRENTLIMIT, pfValue);

	return m_iErrorState;
}

int CDevice::getMaxPWMOutput(int iModuleId, long* piValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601 )
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = readLong(iModuleId, CMDID_GETPARAM, PARID_ACT_MAXPWMOUTPUT, piValue);

	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- modify functions ---------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

void CDevice::setName(const char* acName)
{
	strncpy(m_acName, acName, 128);
}

void CDevice::setInitString(const char* acInitString)
{
	strncpy(m_acInitString, acInitString, 128);
}

int CDevice::setConfig(int iModuleId, unsigned long uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeUnsignedLong(iModuleId, CMDID_SETPARAM, PARID_ACT_CONFIG, uiValue);

	return m_iErrorState;
}

int CDevice::setHomeOffset(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FHOMEOFFSET, fValue);

	return m_iErrorState;
}

int CDevice::setHomeOffsetInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IHOMEOFFSET, iValue);

	return m_iErrorState;
}

int CDevice::setDioData(int iModuleId, unsigned long uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeUnsignedLong(iModuleId, CMDID_SETPARAM, PARID_ACT_DIODATA, uiValue);

	return m_iErrorState;
}

int CDevice::setA0(int iModuleId, short iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeShort(iModuleId, CMDID_SETPARAM, PARID_ACT_A0, iValue);

	return m_iErrorState;
}

int CDevice::setC0(int iModuleId, short iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeShort(iModuleId, CMDID_SETPARAM, PARID_ACT_C0, iValue);

	return m_iErrorState;
}

int CDevice::setDamp(int iModuleId, short iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeShort(iModuleId, CMDID_SETPARAM, PARID_ACT_DAMP, iValue);

	return m_iErrorState;
}

int CDevice::setMinPos(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FMINPOS, fValue);

	return m_iErrorState;
}

int CDevice::setMaxPos(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FMAXPOS, fValue);

	return m_iErrorState;
}

int CDevice::setMaxVel(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FMAXVEL, fValue);

	return m_iErrorState;
}

int CDevice::setMaxAcc(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FMAXACC, fValue);

	return m_iErrorState;
}

int CDevice::setMaxCur(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FMAXCUR, fValue);

	return m_iErrorState;
}

int CDevice::setMaxDeltaPos(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FMAXDELTAPOS, fValue);

	return m_iErrorState;
}

int CDevice::setMinPosInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IMINPOS, iValue);

	return m_iErrorState;
}

int CDevice::setMaxPosInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IMAXPOS, iValue);

	return m_iErrorState;
}

int CDevice::setMaxVelInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IMAXVEL, iValue);

	return m_iErrorState;
}

int CDevice::setMaxAccInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IMAXACC, iValue);

	return m_iErrorState;
}

int CDevice::setMaxDeltaPosInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IMAXDELTAPOS, iValue);

	return m_iErrorState;
}

int CDevice::setHomeVel(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x2518 || (m_auiModuleVersion[i] >= 0x3500 && m_auiModuleVersion[i] < 0x3518))
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FHOMEVEL, fValue);

	return m_iErrorState;
}

int CDevice::setHomeVelInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x2518 || (m_auiModuleVersion[i] >= 0x3500 && m_auiModuleVersion[i] < 0x3518))
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IHOMEVEL, iValue);

	return m_iErrorState;
}

int CDevice::setRampVel(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FRAMPVEL, fValue);

	return m_iErrorState;
}

int CDevice::setRampVelInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IRAMPVEL, iValue);

	return m_iErrorState;
}

int CDevice::setRampAcc(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FRAMPACC, fValue);

	return m_iErrorState;
}

int CDevice::setRampAccInc(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IRAMPACC, iValue);

	return m_iErrorState;
}

int CDevice::setSyncTime(int iModuleId, short iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x3602)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeShort(iModuleId, CMDID_SETPARAM, PARID_ACT_SYNCTIME, iValue);

	return m_iErrorState;
}

int CDevice::setLoadLimit(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x351A)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_LOADLIMIT, iValue);

	return m_iErrorState;
}

int CDevice::setMaxLoadGradient(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x351A)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_MAXLOADGRADIENT, iValue);

	return m_iErrorState;
}

int CDevice::setLoadDeltaTime(int iModuleId, unsigned short iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x351A)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeShort(iModuleId, CMDID_SETPARAM, PARID_ACT_LOADDELTATIME, iValue);

	return m_iErrorState;
}

int CDevice::setDefGearRatio(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_DEF_FGEARRATIO, fValue);

	return m_iErrorState;
}

int CDevice::setDefLinRatio(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_DEF_FLINEARRATIO, fValue);

	return m_iErrorState;
}

int CDevice::setDefCurRatio(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_DEF_FCURRATIO, fValue);

	return m_iErrorState;
}

int CDevice::setDefHomeAcc(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_DEF_FHOMEACC, fValue);

	return m_iErrorState;
}

int CDevice::setModuleSerialNo(int iModuleId, unsigned long uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_DEF_SERIALNO, uiValue);

	return m_iErrorState;
}

int CDevice::setDefIncPerTurn(int iModuleId, unsigned long uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_DEF_INCPERTURN, uiValue);

	return m_iErrorState;
}

int CDevice::setDefBrakeTimeOut(int iModuleId, unsigned short uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeShort(iModuleId, CMDID_SETPARAM, PARID_DEF_BRAKETIMEOUT, uiValue);

	return m_iErrorState;
}

int CDevice::setDefAddress(int iModuleId, unsigned char uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeChar(iModuleId, CMDID_SETPARAM, PARID_DEF_ADDRESS, uiValue);

	return m_iErrorState;
}

int CDevice::setDefCANBaudRate(int iModuleId, unsigned char uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeChar(iModuleId, CMDID_SETPARAM, PARID_DEF_CANBAUDRATE, uiValue);

	return m_iErrorState;
}

int CDevice::setDefRSBaudRate(int iModuleId, unsigned char uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeChar(iModuleId, CMDID_SETPARAM, PARID_DEF_RSBAUDRATE, uiValue);

	return m_iErrorState;
}

int CDevice::setDefSetup(int iModuleId, unsigned long uiValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_DEF_SETUP, uiValue);

	return m_iErrorState;
}

int CDevice::setMinLogicVoltage(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_MINLOGIC, fValue);

	return m_iErrorState;
}

int CDevice::setMaxLogicVoltage(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_MAXLOGIC, fValue);

	return m_iErrorState;
}

int CDevice::setMinMotorVoltage(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_MINMOTOR, fValue);

	return m_iErrorState;
}

int CDevice::setMaxMotorVoltage(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_MAXMOTOR, fValue);

	return m_iErrorState;
}

int CDevice::setNominalMotorCurrent(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_NOMCUR, fValue);

	return m_iErrorState;
}

int CDevice::setMaximumMotorCurrent(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_HMAXCUR, fValue);

	return m_iErrorState;
}

int CDevice::setLogicUndershootTime(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_LOGICUNDERSHOOT, iValue);

	return m_iErrorState;
}

int CDevice::setLogicOvershootTime(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_LOGICOVERSHOOT, iValue);

	return m_iErrorState;
}

int CDevice::setMotorUndershootTime(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_MOTORUNDERSHOOT, iValue);

	return m_iErrorState;
}

int CDevice::setMotorOvershootTime(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_MOTOROVERSHOOT, iValue);

	return m_iErrorState;
}

int CDevice::setNomCurOvershootTime(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_NOMCUROVERSHOOT, iValue);

	return m_iErrorState;
}

int CDevice::setHMaxCurOvershootTime(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_HMAXCUROVERSHOOT, iValue);

	return m_iErrorState;
}

int CDevice::setKpPWMLimit(int iModuleId, long iValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_KP_PWMLIM, iValue);

	return m_iErrorState;
}

int CDevice::setCurrentLimit(int iModuleId, float fValue)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	for(int i = 0; i < m_iModuleCount; i++)
	{
		if(m_aiModuleId[i] == iModuleId)
		{
			if(m_auiModuleVersion[i] < 0x4601)
			{
				warning("module version does not support function");
				m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
				return m_iErrorState;
			}
		}
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_CURRENTLIMIT, fValue);

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

int CDevice::updateModuleIdMap()
{
	unsigned short uiVersion;
	std::vector<float> afData;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_aiModuleId.clear();
	m_auiModuleVersion.clear();

	m_iModuleCount = m_iModuleCountMax;	// timout = m_uiTimeOut + 10 * m_iModuleCount for RS232
	for(int i = 1; i <= m_iModuleCountMax; i++)
	{
		debug(0,"searching for module %i", i);
		m_iErrorState = getModuleVersion(i, &uiVersion);
		if(m_iErrorState == 0)
		{
			m_aiModuleId.push_back(i);
			m_auiModuleVersion.push_back(uiVersion);
			debug(0,"found module with ID %i and Version %x", i, uiVersion);
		}
		else
			m_iErrorState = 0;
	}
	m_iModuleCount = m_aiModuleId.size();
	m_iErrorState = 0;
	return m_iModuleCount;
}

int CDevice::homeModule(int iModuleId)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeCommand(iModuleId, CMDID_HOME);

	return m_iErrorState;
}

int CDevice::haltModule(int iModuleId)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeCommand(iModuleId, CMDID_HALT);

	return m_iErrorState;
}

int CDevice::resetModule(int iModuleId)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeCommand(iModuleId, CMDID_RESET);

	return m_iErrorState;
}

int CDevice::recalcPIDParams(int iModuleId)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeCommand(iModuleId, CMDID_RECALCPID);

	return m_iErrorState;
}

int CDevice::saveParameters(int iModuleId)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeCommand(iModuleId, CMDID_SAVEPARAMS);

	return m_iErrorState;
}

int CDevice::movePos(int iModuleId, float fPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETMOVE, PARID_MOVE_FRAMP, fPos);

	return m_iErrorState;
}

int CDevice::moveRamp(int iModuleId, float fPos, float fVel, float fAcc)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FRAMPVEL, fVel);
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FRAMPACC, fAcc);
	m_iErrorState = writeFloat(iModuleId, CMDID_SETMOVE, PARID_MOVE_FRAMP, fPos);

	return m_iErrorState;
}

int CDevice::moveVel(int iModuleId, float fVel)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETMOVE, PARID_MOVE_FVEL, fVel);

	return m_iErrorState;
}

int CDevice::moveCur(int iModuleId, float fCur)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloat(iModuleId, CMDID_SETMOVE, PARID_MOVE_FCUR, fCur);

	return m_iErrorState;
}

int CDevice::moveStep(int iModuleId, float fPos, unsigned short uiTime)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeFloatShort(iModuleId, CMDID_SETMOVE, PARID_MOVE_FSTEP, fPos, uiTime);

	return m_iErrorState;
}

int CDevice::movePosInc(int iModuleId, long iPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETMOVE, PARID_MOVE_IRAMP, iPos);

	return m_iErrorState;
}

int CDevice::moveRampInc(int iModuleId, long iPos, long iVel, long iAcc)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IRAMPVEL, iVel);
	m_iErrorState = writeLong(iModuleId, CMDID_SETPARAM, PARID_ACT_IRAMPACC, iAcc);
	m_iErrorState = writeLong(iModuleId, CMDID_SETMOVE, PARID_MOVE_IRAMP, iPos);

	return m_iErrorState;
}

int CDevice::moveVelInc(int iModuleId, long iVel)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETMOVE, PARID_MOVE_IVEL, iVel);

	return m_iErrorState;
}

int CDevice::moveCurInc(int iModuleId, long iCur)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLong(iModuleId, CMDID_SETMOVE, PARID_MOVE_ICUR, iCur);

	return m_iErrorState;
}

int CDevice::moveStepInc(int iModuleId, long iPos, unsigned short uiTime)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_iErrorState = writeLongShort(iModuleId, CMDID_SETMOVE, PARID_MOVE_ISTEP, iPos, uiTime);

	return m_iErrorState;
}

int CDevice::movePosExtended(int iModuleId, float fPos, 
	unsigned long* puiState, unsigned char* pucDio, float* pfPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	unsigned char ucState = 0;
	m_iErrorState = writeFloatReadFloatUnsignedChars(iModuleId, CMDID_SETMOVE, PARID_MOVE_FRAMP_EXT, fPos, pfPos, &ucState, pucDio);
	if(m_iErrorState == 0)
		charStateToLongState(ucState, puiState);

	return m_iErrorState;
}

int CDevice::moveRampExtended(int iModuleId, float fPos, float fVel, float fAcc, 
	unsigned long* puiState, unsigned char* pucDio, float* pfPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	unsigned char ucState = 0;
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FRAMPVEL, fVel);
	m_iErrorState = writeFloat(iModuleId, CMDID_SETPARAM, PARID_ACT_FRAMPACC, fAcc);
	m_iErrorState = writeFloatReadFloatUnsignedChars(iModuleId, CMDID_SETMOVE, PARID_MOVE_FRAMP_EXT, fPos, pfPos, &ucState, pucDio);
	if(m_iErrorState == 0)
		charStateToLongState(ucState, puiState);

	return m_iErrorState;
}

int CDevice::moveVelExtended(int iModuleId, float fVel, 
	unsigned long* puiState, unsigned char* pucDio, float* pfPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	unsigned char ucState = 0;
	m_iErrorState = writeFloatReadFloatUnsignedChars(iModuleId, CMDID_SETMOVE, PARID_MOVE_FVEL_EXT, fVel, pfPos, &ucState, pucDio);
	if(m_iErrorState == 0)
		charStateToLongState(ucState, puiState);

	return m_iErrorState;
}

int CDevice::moveCurExtended(int iModuleId, float fCur, 
	unsigned long* puiState, unsigned char* pucDio, float* pfPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	unsigned char ucState = 0;
	m_iErrorState = writeFloatReadFloatUnsignedChars(iModuleId, CMDID_SETMOVE, PARID_MOVE_FCUR_EXT, fCur, pfPos, &ucState, pucDio);
	if(m_iErrorState == 0)
		charStateToLongState(ucState, puiState);

	return m_iErrorState;
}

int CDevice::moveStepExtended(int iModuleId, float fPos, unsigned short uiTime, 
	unsigned long* puiState, unsigned char* pucDio, float* pfPos)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	unsigned char ucState = 0;
	m_iErrorState = writeFloatShortReadFloatUnsignedChars(iModuleId, CMDID_SETMOVE, PARID_MOVE_FSTEP_EXT, fPos, uiTime, pfPos, &ucState, pucDio);
	if(m_iErrorState == 0)
		charStateToLongState(ucState, puiState);
	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- wait functions ------------------------------------------------------ ;
//                                                                            ;
// ========================================================================== ;

int CDevice::waitForHomeEnd(int iModuleId, unsigned long uiTimeOut)
{
	int iRetVal;
	unsigned long uiTime, uiState;
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_clTimer.start();
	while(1)
	{
		Sleep(1);
		iRetVal = getModuleState(iModuleId, &uiState);
		if (iRetVal < 0)
		{
			debug(1,"com error in waitForHomeEnd()");
			break;
		}
		if (uiState & STATEID_MOD_ERROR)
		{
			debug(1, "module error in waitForHomeEnd()");
			iRetVal=ERRID_DEV_MODULEERROR;
			break;
		}
		if ((uiState & STATEID_MOD_HOME) && (uiState & STATEID_MOD_RAMP_END))
		{
			debug(2, "reached home position in waitForHomeEnd()");
			iRetVal=0;
			break;
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForHomeEnd()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			break;
		}
	}
	return iRetVal;
}

int CDevice::waitForMotionEnd(int iModuleId, unsigned long uiTimeOut)
{
	int iRetVal;
	unsigned long uiTime, uiState;
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_clTimer.start();
	while(1)
	{
		Sleep(1);
		iRetVal = getModuleState(iModuleId, &uiState);
		if (iRetVal < 0)
		{
			debug(1,"com error in waitForMotionEnd()");
			break;
		}
		if (uiState & STATEID_MOD_ERROR)
		{
			debug(1, "module error in waitForMotionEnd()");
			iRetVal=ERRID_DEV_MODULEERROR;
			break;
		}
		if (!(uiState & STATEID_MOD_MOTION))
		{
			debug(2, "finished motion in waitForMotionEnd()");
			iRetVal=0;
			break;
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForMotionEnd()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			break;
		}
	}
	return iRetVal;
}

int CDevice::waitForRampEnd(int iModuleId, unsigned long uiTimeOut)
{
	int iRetVal;
	unsigned long uiTime, uiState;
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_clTimer.start();
	while(1)
	{
		Sleep(1);
		iRetVal = getModuleState(iModuleId, &uiState);
		if (iRetVal < 0)
		{
			debug(1,"com error in waitForRampEnd()");
			break;
		}
		if (uiState & STATEID_MOD_ERROR)
		{
			debug(1, "module error in waitForRampEnd()");
			iRetVal=ERRID_DEV_MODULEERROR;
			break;
		}
		if (uiState & STATEID_MOD_RAMP_END)
		{
			debug(2, "reached ramp end position in waitForRampEnd()");
			iRetVal=0;
			break;
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForRampEnd()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			break;
		}
	}
	return iRetVal;
}

int CDevice::waitForRampDec(int iModuleId, unsigned long uiTimeOut)
{
	int iRetVal;
	unsigned long uiTime, uiState;
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_clTimer.start();
	while(1)
	{
		Sleep(1);
		iRetVal = getModuleState(iModuleId, &uiState);
		if (iRetVal < 0)
		{
			debug(1,"com error in waitForRampDec()");
			break;
		}
		if (uiState & STATEID_MOD_ERROR)
		{
			debug(1, "module error in waitForRampDec()");
			iRetVal=ERRID_DEV_MODULEERROR;
			break;
		}
		if (uiState & STATEID_MOD_RAMP_DEC)
		{
			debug(2, "reached ramp end position in waitForRampDec()");
			iRetVal=0;
			break;
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForRampDec()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			break;
		}
	}
	return iRetVal;
}

int CDevice::waitForRampSteady(int iModuleId, unsigned long uiTimeOut)
{
	int iRetVal;
	unsigned long uiTime, uiState;
	if(iModuleId < 0 || iModuleId > m_iModuleCountMax)
	{
		warning("wrong module id");
		m_iErrorState = ERRID_DEV_WRONGMODULEID;
		return m_iErrorState;
	}
	m_clTimer.start();
	while(1)
	{
		Sleep(1);
		iRetVal = getModuleState(iModuleId, &uiState);
		if (iRetVal < 0)
		{
			debug(1,"com error in waitForRampSteady()");
			break;
		}
		if (uiState & STATEID_MOD_ERROR)
		{
			debug(1, "module error in waitForRampSteady()");
			iRetVal=ERRID_DEV_MODULEERROR;
			break;
		}
		if (uiState & STATEID_MOD_RAMP_STEADY)
		{
			debug(2, "reached ramp end position in waitForRampSteady()");
			iRetVal=0;
			break;
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForRampSteady()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			break;
		}
	}
	return iRetVal;
}

int CDevice::waitForHomeEndAll(unsigned long uiTimeOut)
{
	int i, iRetVal = ERRID_DEV_NOMODULES;
	bool bExitFlag = false;
	unsigned long uiTime, uiState;
	m_clTimer.start();
	do
	{
		Sleep(1);
		bExitFlag = true;
		for(i = 0; i < m_iModuleCount;i++)
		{
			iRetVal = getModuleState(m_aiModuleId[i], &uiState);
			if (iRetVal < 0)
			{
				debug(1,"com error in waitForHomeEndAll()");
				return iRetVal;
			}
			if (uiState & STATEID_MOD_ERROR)
			{
				debug(1, "module error in waitForHomeEndAll()");
				iRetVal=ERRID_DEV_MODULEERROR;
				return iRetVal;
			}
			if (!(uiState & STATEID_MOD_HOME) || !(uiState & STATEID_MOD_RAMP_END))
			{
				debug(2, "module %i not home end in waitForHomeEndAll()", m_aiModuleId[i]);
				bExitFlag = false;
				break;
			}
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForHomeEndAll()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			return iRetVal;
		}
	}while(!bExitFlag);
	return iRetVal;
}

int CDevice::waitForMotionEndAll(unsigned long uiTimeOut)
{
	int i, iRetVal = ERRID_DEV_NOMODULES;
	bool bExitFlag = false;
	unsigned long uiTime, uiState;
	m_clTimer.start();
	do
	{
		Sleep(1);
		bExitFlag = true;
		for(i = 0; i < m_iModuleCount;i++)
		{
			iRetVal = getModuleState(m_aiModuleId[i], &uiState);
			if (iRetVal < 0)
			{
				debug(1,"com error in waitForMotionEndAll()");
				return iRetVal;
			}
			if (uiState & STATEID_MOD_ERROR)
			{
				debug(1, "module error in waitForMotionEndAll()");
				iRetVal=ERRID_DEV_MODULEERROR;
				return iRetVal;
			}
			if (uiState & STATEID_MOD_MOTION)
			{
				debug(2, "module %i not motion end in waitForMotionEndAll()", m_aiModuleId[i]);
				bExitFlag = false;
				break;
			}
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForMotionEndAll()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			return iRetVal;
		}
	}while(!bExitFlag);
	return iRetVal;
}

int CDevice::waitForRampEndAll(unsigned long uiTimeOut)
{
	int i, iRetVal = ERRID_DEV_NOMODULES;
	bool bExitFlag = false;
	unsigned long uiTime, uiState;
	m_clTimer.start();
	do
	{
		Sleep(1);
		bExitFlag = true;
		for(i = 0; i < m_iModuleCount;i++)
		{
			iRetVal = getModuleState(m_aiModuleId[i], &uiState);
			if (iRetVal < 0)
			{
				debug(1,"com error in waitForRampEndAll()");
				return iRetVal;
			}
			if (uiState & STATEID_MOD_ERROR)
			{
				debug(1, "module error in waitForRampEndAll()");
				iRetVal=ERRID_DEV_MODULEERROR;
				return iRetVal;
			}
			if (!(uiState & STATEID_MOD_RAMP_END))
			{
				debug(2, "module %i not ramp end in waitForRampEndAll()", m_aiModuleId[i]);
				bExitFlag = false;
				break;
			}
		}
		m_clTimer.stop();
		uiTime = (unsigned long)(1000 * m_clTimer.executionTime());
		if (uiTime > uiTimeOut)
		{
			debug(1, "timeout in waitForRampEndAll()");
			iRetVal=ERRID_DEV_WAITTIMEOUT;
			return iRetVal;
		}
	}while(!bExitFlag);
	return iRetVal;
}

int CDevice::waitForStartMotionAll()
{ 
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = ERRID_DEV_FUNCTIONNOTAVAILABLE;
	return m_iErrorState;
}


// ========================================================================== ;
//                                                                            ;
// ---- broadcast functions -------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

int CDevice::homeAll(void)
{ 
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = writeAll( CMDID_HOME, 0 );
	return m_iErrorState;
}

int CDevice::resetAll(void)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = writeAll( CMDID_RESET, 0 );
	return m_iErrorState;
}

int CDevice::haltAll(void)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = writeAll( CMDID_HALT, 0 );
	return m_iErrorState;
}

int CDevice::serveWatchdogAll(void)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = writeAll( CMDID_WATCHDOG, 0 );
	return m_iErrorState;
}

int CDevice::setBaudRateAll(unsigned char ucBaudRateId)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = writeAll( CMDID_BAUDRATE, ucBaudRateId );
	reinit(ucBaudRateId);
	return m_iErrorState;
}

int CDevice::startMotionAll(void)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = writeAll( CMDID_STARTMOVE, 0 );
	return m_iErrorState;
}

int CDevice::savePosAll(void)
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = writeAll( CMDID_SAVEPOS, 0 );
	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- Special functions for internal use ---------------------------------- ;
//                                                                            ;
// ========================================================================== ;
int CDevice::xmit8Bytes(int iModuleId, void* pBytes )
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = write8Bytes(iModuleId, false, pBytes );
	return m_iErrorState;
}

int CDevice::xack8Bytes(int iModuleId, void* pBytes )
{
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = write8Bytes( iModuleId, true, pBytes );
	return m_iErrorState;
}

int CDevice::doInternal(int iModuleId, void* pBytes )
{	unsigned char toSend[8];
	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	toSend[0] = 0x04;
	memcpy( &toSend[1], pBytes, 7 );
	m_iErrorState = write8Bytes( iModuleId, true, toSend );
	memcpy( pBytes, &toSend[1], 7 );
	return m_iErrorState;
}

int CDevice::getStateInternal(int iModuleId, unsigned long* pStat )
{	m_iErrorState = 0;
	if(m_bInitFlag == false)
	{
		warning("device not initialized");
		m_iErrorState = ERRID_DEV_NOTINITIALIZED;
		return m_iErrorState;
	}
	m_iErrorState = getUnsignedLong( iModuleId, pStat );
	return m_iErrorState;
}

// ========================================================================== ;
//                                                                            ;
// ---- Init functions ------------------------------------------------------ ;
//                                                                            ;
// ========================================================================== ;

CDevice* newDevice(const char* acInitString)
{
	char* pcToken;
	char acString[128];

	strncpy(acString,acInitString,128);
	pcToken = strtok( acString, ":" );
	if( !pcToken )
        {
                printf("CDevice* newDevice(const char* acInitString): wrong format, no ':' found!\n");
		return NULL;
        }

	if( strcmp( pcToken, "RS232" ) == 0 )
	{
		return new CRS232Device();
	}
#ifdef USE_PCAN
	if( strcmp( pcToken, "PCAN" ) == 0 )
	{
		return new CPCanDevice();
	}
#endif
#ifdef USE_ESD
	if( strcmp( pcToken, "ESD" ) == 0 )
	{
		return new CESDDevice();
	}
#endif
#if defined(_WIN32)	
	if( strcmp( pcToken, "CP5X11" ) == 0 )
	{
		return new CCP5X11Device();
	}
#endif
        printf("CDevice* newDevice(const char* acInitString): wrong format, no device found!\n");
	return NULL;
}
