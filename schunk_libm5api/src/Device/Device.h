
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


#ifndef CDEVICE_H
#define CDEVICE_H

#include "../Util/Message.h"
#include "../Util/StopWatch.h"
#include "../ComDef/DeviceErrors.h"
#include "../ComDef/ModuleErrors.h"
#include "../ComDef/IOErrors.h"
#include "../Device/ProtocolCommands.h"

class CDevice : public CMessage
{
	private:

		// ---- private data ---------------------------------------------------- ;
	
		// ---- private auxiliary functions ------------------------------------- ;

	protected:

		// ---- protected data -------------------------------------------------- ;

			bool m_bInitFlag;
			char m_acName[128];
			char m_acInitString[128];
			char m_acRevision[20];
			int m_iBaudRate;
			int m_iModuleCount;
			int m_iModuleCountMax;
			std::vector<int> m_aiModuleId;
			std::vector<unsigned short> m_auiModuleVersion;
			CStopWatch m_clTimer;
			int m_iErrorState;

		// ---- protected auxiliary functions ----------------------------------- ;

			virtual int reinit(unsigned char ucBaudRateId) = 0;
			virtual int write8Bytes(int iModuleId, bool ack, void* pBytes ) = 0; 
			virtual int getUnsignedLong(int iModuleId, unsigned long* puiData) = 0;

			virtual int readChar(int iModuleId, int iCommandId, int iParameterId, char* pcData) = 0;
			virtual int readUnsignedChar(int iModuleId, int iCommandId, int iParameterId, unsigned char* pucData) = 0;
			virtual int readShort(int iModuleId, int iCommandId, int iParameterId, short* piData) = 0;
			virtual int readUnsignedShort(int iModuleId, int iCommandId, int iParameterId, unsigned short* puiData) = 0;
			virtual int readLong(int iModuleId, int iCommandId, int iParameterId, long* piData) = 0;
			virtual int readUnsignedLong(int iModuleId, int iCommandId, int iParameterId, unsigned long* puiData) = 0;
			virtual int readFloat(int iModuleId, int iCommandId, int iParameterId, float* pfData) = 0;

			virtual int readLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long* piData, unsigned char* pucData1, unsigned char* pucData2) = 0;
			virtual int readFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float* pfData, unsigned char* pucData1, unsigned char* pucData2) = 0;

			virtual int writeChar(int iModuleId, int iCommandId, int iParameterId, char cData) = 0;
			virtual int writeUnsignedChar(int iModuleId, int iCommandId, int iParameterId, unsigned char ucData) = 0;
			virtual int writeShort(int iModuleId, int iCommandId, int iParameterId, short iData) = 0;
			virtual int writeUnsignedShort(int iModuleId, int iCommandId, int iParameterId, unsigned short uiData) = 0;
			virtual int writeLong(int iModuleId, int iCommandId, int iParameterId, long iData) = 0;
			virtual int writeUnsignedLong(int iModuleId, int iCommandId, int iParameterId, unsigned long uiData) = 0;
			virtual int writeFloat(int iModuleId, int iCommandId, int iParameterId, float fData) = 0;

			virtual int writeAll(int iCommandId, int iParameterId) = 0;
			virtual int writeCommand(int iModuleId, int iCommandId) = 0;
			virtual int writeLongShort(int iModuleId, int iCommandId, int iParameterId, long iData, short iTime) = 0;
			virtual int writeFloatShort(int iModuleId, int iCommandId, int iParameterId, float fData, short iTime) = 0;

			virtual int writeShortReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, short iData, long* piData, unsigned char* pucData1, unsigned char* pucData2) = 0;
			virtual int writeLongReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long iData, long* piData, unsigned char* pucData1, unsigned char* pucData2) = 0;
			virtual int writeLongShortReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long iData1, short iData2, long* piData, unsigned char* pucData1, unsigned char* pucData2) = 0;
			virtual int writeFloatReadFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float fData, float* pfData, unsigned char* pucData1, unsigned char* pucData2) = 0;
			virtual int writeFloatShortReadFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float fData, short iData, float* pfData, unsigned char* pucData1, unsigned char* pucData2) = 0;

			void charStateToLongState(unsigned char uiShort, unsigned long* puiState);

	public:

		// ---- public data ----------------------------------------------------- ;

	
		// ---- constructors / destructor --------------------------------------- ;

			/// default constructor
			CDevice(void);
			/// copy constructor
			CDevice(const CDevice& rclDevice);
			/// destructor
			virtual ~CDevice();

		// ---- operators ------------------------------------------------------- ;
		
			// assignment operator
			CDevice& operator=(const CDevice& rclDevice);

		// ---- query functions ------------------------------------------------- ;

			/// gets the init flag
			bool getInitFlag();
			/// gets the revision string of the class
			const char* getRevision();
			/// gets the name string of the open device
			const char* getName();
			/// gets the init string of the device
			const char* getInitString();
			/// gets the baud rate of the device
			int getBaudRate(void);
			/// gets the number of modules of the device
			int getModuleCount(void);
			/// gets the module ID map of the open device
			int getModuleIdMap(std::vector<int>& raiModuleId);

			int getModuleState(int iModuleId, unsigned long* puiState);
			int getModuleType(int iModuleId, unsigned char* pucValue);
			int getModuleVersion(int iModuleId, unsigned short* puiValue);
			int getModuleSerialNo(int iModuleId, unsigned long* puiValue);

			int getDefConfig(int iModuleId, unsigned long* puiValue);
			int getDefSetup(int iModuleId, unsigned long* puiValue);
			int getDefBaudRate(int iModuleId, unsigned char* pucValue);
			int getDefBurnCount(int iModuleId, unsigned char* pucValue);
			int getDefGearRatio(int iModuleId, float* pfValue);
			int getDefLinearRatio(int iModuleId, float* pfValue);
			int getDefCurOffset(int iModuleId, float* pfValue);
			int getDefCurRatio(int iModuleId, float* pfValue);
			int getDefBrakeTimeOut(int iModuleId, unsigned short* puiValue);
			int getDefIncPerTurn(int iModuleId, unsigned long* puiValue);
			int getDefDioData(int iModuleId, unsigned long* puiValue);
			int getDefA0(int iModuleId, short* piValue);
			int getDefC0(int iModuleId, short* piValue);
			int getDefDamp(int iModuleId, short* piValue);
			int getDefHomeOffset(int iModuleId, float* pfValue);
			int getDefHomeVel(int iModuleId, float* pfValue);
			int getDefMinPos(int iModuleId, float* pfValue);
			int getDefMaxPos(int iModuleId, float* pfValue);
			int getDefMaxVel(int iModuleId, float* pfValue);
			int getDefMaxAcc(int iModuleId, float* pfValue);
			int getDefMaxCur(int iModuleId, float* pfValue);
			int getDefMaxDeltaPos(int iModuleId, float* pfValue);

			int getConfig(int iModuleId, unsigned long* puiValue);
			int getIncRatio(int iModuleId, float* pValue );
			int getStateDioPos(int iModuleId, unsigned long* puiState, 
				unsigned char* pucDio, float* pfPos);
			int getDioData(int iModuleId, unsigned long* puiValue);
			int getA0(int iModuleId, short* piValue);
			int getC0(int iModuleId, short* piValue);
			int getDamp(int iModuleId, short* piValue);
			int getHomeOffset(int iModuleId, float* pValue );
			int getHomeOffsetInc(int iModuleId, long* piValue );
			int getHomeVel(int iModuleId, float* pfValue );
			int getHomeVelInc(int iModuleId, long* piValue);
			int getPos(int iModuleId, float* pfPos);
			int getPosInc(int iModuleId, long* piValue);
			int getPosCountInc(int iModuleId, long* piValue);
			int getVel(int iModuleId, float* pfVel);
			int getVelInc(int iModuleId, long* piValue);
			int getIPolVel(int iModuleId, float* pValue );
			int getCur(int iModuleId, float* pfCur);
			int getCurInc(int iModuleId, short* piValue);
			int getMinPos(int iModuleId, float* pfValue);
			int getMinPosInc(int iModuleId, long* piValue);
			int getMaxPos(int iModuleId, float* pfValue);
			int getMaxPosInc(int iModuleId, long* piValue);
			int getMaxVel(int iModuleId, float* pfValue);
			int getMaxVelInc(int iModuleId, long* piValue);
			int getMaxAcc(int iModuleId, float* pfValue);
			int getMaxAccInc(int iModuleId, long* piValue);
			int getMaxCur(int iModuleId, float* pfValue);
			int getDeltaPos(int iModuleId, float* pfValue);
			int getDeltaPosInc(int iModuleId, long* piValue);
			int getMaxDeltaPos(int iModuleId, float* pfValue);
			int getMaxDeltaPosInc(int iModuleId, long* piValue);
			int getSavePos(int iModuleId, float* pfValue);
			int getSyncTime(int iModuleId, short* piValue);
			int getRawMotorCurrent(int iModuleId, short* piValue);
			int getRawMotorSupply(int iModuleId, short* piValue);
			int getRawTemperature(int iModuleId, short* piValue);
			int getRawLogicSupply(int iModuleId, short* piValue);
			int getLoadLimit(int iModuleId, long* piValue);
			int getMaxLoadGradient(int iModuleId, long* piValue);
			int getLoadDeltaTime(int iModuleId, unsigned short* piValue);

			int getMotorCurrent(int iModuleId, float* pfValue);
			int getMotorSupply(int iModuleId, float* pfValue);
			int getTemperature(int iModuleId, float* pfValue);
			int getLogicSupply(int iModuleId, float* pfValue);

			int getMinLogicVoltage(int iModuleId, float* pfValue);
			int getMaxLogicVoltage(int iModuleId, float* pfValue);
			int getMinMotorVoltage(int iModuleId, float* pfValue);
			int getMaxMotorVoltage(int iModuleId, float* pfValue);
			int getNominalMotorCurrent(int iModuleId, float* pfValue);
			int getMaximumMotorCurrent(int iModuleId, float* pfValue);
			int getLogicUndershootTime(int iModuleId, long* piValue);
			int getLogicOvershootTime(int iModuleId, long* piValue);
			int getMotorUndershootTime(int iModuleId, long* piValue);
			int getMotorOvershootTime(int iModuleId, long* piValue);
			int getNomCurOvershootTime(int iModuleId, long* piValue);
			int getHMaxCurOvershootTime(int iModuleId, long* piValue);

			int getDefCANBaudRate(int iModuleId, unsigned char* pucValue);
			int getDefRSBaudRate(int iModuleId, unsigned char* pucValue);

			int getKpPWMLimit(int iModuleId, long* piValue);
			int getCurrentLimit(int iModuleId, float* pfValue);
			int	getMaxPWMOutput(int iModuleId, long* piValue);

			virtual int getDataDLR_FTS(std::vector<float>& rafData, long* piState) = 0;
			virtual int getDataSCHUNK_FTC(int iModuleId, int iChannelTypeId, std::vector<float>& rafData, short* piState) = 0;
			virtual int getDataMP55_IO(int iModuleId, float* pfData) = 0;
			virtual int getDataMP55_IO_fast(int iModuleId, float* pfData) = 0;
			virtual int getCanOpenRawAbsEnc(int iModuleId, short* piValue) = 0;


		// ---- modify functions ------------------------------------------------ ;

			/// sets the baud rate of the device
			void setName(const char* acNameString);
			void setInitString(const char* acInitString);

			int setConfig(int iModuleId, unsigned long puiValue);
			int setDioData(int iModuleId, unsigned long uiValue);
			int setA0(int iModuleId, short iValue);
			int setC0(int iModuleId, short iValue);
			int setDamp(int iModuleId, short iValue);
			int setHomeOffset(int iModuleId, float fValue );
			int setHomeOffsetInc(int iModuleId, long iValue );
			int setHomeVel(int iModuleId, float fValue);			
			int setHomeVelInc(int iModuleId, long iValue);
			int setRampVel(int iModuleId, float fValue );
			int setRampAcc(int iModuleId, float fValue );
			int setRampVelInc(int iModuleId, long iValue );
			int setRampAccInc(int iModuleId, long iValue );
			int setMinPos(int iModuleId, float fValue);
			int setMinPosInc(int iModuleId, long iValue);
			int setMaxPos(int iModuleId, float fValue);
			int setMaxPosInc(int iModuleId, long iValue);
			int setMaxVel(int iModuleId, float fValue);
			int setMaxVelInc(int iModuleId, long iValue);
			int setMaxAcc(int iModuleId, float fValue);
			int setMaxAccInc(int iModuleId, long iValue);
			int setMaxCur(int iModuleId, float fValue);
			int setMaxDeltaPos(int iModuleId, float fValue);
			int setMaxDeltaPosInc(int iModuleId, long iValue);
			int setSyncTime(int iModuleId, short iValue);
			int setLoadLimit(int iModuleId, long iValue);
			int setMaxLoadGradient(int iModuleId, long iValue);
			int setLoadDeltaTime(int iModuleId, unsigned short iValue);
			
			int setDefGearRatio(int iModuleId, float fValue);
			int setDefLinRatio(int iModuleId, float fValue);
			int setDefCurRatio(int iModuleId, float fValue);
			int setDefHomeAcc(int iModuleId, float fValue);
			int setModuleSerialNo(int iModuleId, unsigned long uiValue);
			int setDefIncPerTurn(int iModuleId, unsigned long uiValue);
			int setDefBrakeTimeOut(int iModuleId, unsigned short uiValue);
			int setDefAddress(int iModuleId, unsigned char uiValue);
			int setDefCANBaudRate(int iModuleId, unsigned char uiValue);
			int setDefRSBaudRate(int iModuleId, unsigned char uiValue);
			int setDefSetup(int iModuleId, unsigned long uiValue);

			int setMinLogicVoltage(int iModuleId, float fValue);
			int setMaxLogicVoltage(int iModuleId, float fValue);
			int setMinMotorVoltage(int iModuleId, float fValue);
			int setMaxMotorVoltage(int iModuleId, float fValue);
			int setNominalMotorCurrent(int iModuleId, float fValue);
			int setMaximumMotorCurrent(int iModuleId, float fValue);
			int setLogicUndershootTime(int iModuleId, long iValue);
			int setLogicOvershootTime(int iModuleId, long iValue);
			int setMotorUndershootTime(int iModuleId, long iValue);
			int setMotorOvershootTime(int iModuleId, long iValue);
			int setNomCurOvershootTime(int iModuleId, long iValue);
			int setHMaxCurOvershootTime(int iModuleId, long iValue);

			int setKpPWMLimit(int iModuleId, long iValue);
			int setCurrentLimit(int iModuleId, float fValue);

			virtual int setNullSCHUNK_FTC(int iModuleId, short* piState) = 0;
			virtual int setTaraMP55_IO(int iModuleId, float fTara) = 0;
			virtual int setInitMP55_IO_fast(int iModuleId) = 0;

		// ---- I/O functions --------------------------------------------------- ;

		// ---- exec functions -------------------------------------------------- ;

			virtual int init() = 0;
			virtual int init(const char* acInitString) = 0;
			virtual int exit() = 0;

			virtual int initDLR_FTS() = 0;
			int updateModuleIdMap();

			int homeModule(int iModuleId);
			int resetModule(int iModuleId);
			int haltModule(int iModuleId);
			int recalcPIDParams(int iModuleId);
			int saveParameters(int iModuleId);

			int movePos(int iModuleId, float fPos);
			int movePosInc(int iModuleId, long iPos);
			int movePosExtended(int iModuleId, float fPos, 
				unsigned long* puiState, unsigned char* pucDio, float* pfPos);
			int moveRamp(int iModuleId, float fPos, float fVel, float fAcc);
			int moveRampInc(int iModuleId, long iPos, long iVel, long iAcc);
			int moveRampExtended(int iModuleId, float fPos, float fVel, float fAcc, 
				unsigned long* puiState, unsigned char* pucDio, float* pfPos);
			int moveVel(int iModuleId, float fVel);
			int moveVelInc(int iModuleId, long iVel);
			int moveVelExtended(int iModuleId, float fCur, 
				unsigned long* puiState, unsigned char* pucDio, float* pfPos);
			int moveCur(int iModuleId, float fCur);
			int moveCurInc(int iModuleId, long iCur);
			int moveCurExtended(int iModuleId, float fCur, 
				unsigned long* puiState, unsigned char* pucDio, float* pfPos);
			int moveStep(int iModuleId, float fPos, unsigned short uiTime);
			int moveStepInc(int iModuleId, long iPos, unsigned short uiTime);
			int moveStepExtended(int iModuleId, float fPos, unsigned short uiTime, 
				unsigned long* puiState, unsigned char* pucDio, float* pfPos);

		// ---- wait functions -------------------------------------------------- ;

			int waitForHomeEnd(int iModuleId, unsigned long uiTimeOut = 60000);
			int waitForMotionEnd(int iModuleId, unsigned long uiTimeOut = 60000);
			int waitForRampEnd(int iModuleId, unsigned long uiTimeOut = 60000);
			int waitForRampDec(int iModuleId, unsigned long uiTimeOut = 60000);
			int waitForRampSteady(int iModuleId, unsigned long uiTimeOut = 60000);
			int waitForHomeEndAll(unsigned long uiTimeOut = 60000);
			int waitForMotionEndAll(unsigned long uiTimeOut = 60000);
			int waitForRampEndAll(unsigned long uiTimeOut = 60000);
			virtual int waitForStartMotionAll();

		// ---- broadcast functions -------------------------------------------------- ;

			int homeAll(void);
			int resetAll(void);
			int haltAll(void);
			int serveWatchdogAll(void);
			int setBaudRateAll(unsigned char ucBaudRateId);
			int startMotionAll(void);
			int savePosAll(void);

		// ---- Special functions -------------------------------------------------- ;

			int xmit8Bytes(int iModuleId, void* pBytes );
			int xack8Bytes(int iModuleId, void* pBytes );
			int doInternal(int iModuleId, void* pBytes );
			int getStateInternal(int iModuleId, unsigned long* pBytes );

};

CDevice* newDevice(const char* acInitString);

#endif
