
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


#ifndef CPROTOCOLDEVICE_H
#define CPROTOCOLDEVICE_H

#include "../Device/Device.h"
#include "../Device/ProtocolMessage.h"
#include "../Util/Random.h"
#include "../Util/StopWatch.h"

#define READ 1
#define WRITE 0

class CProtocolDevice : public CDevice  
{
	private:

		// ---- private data ---------------------------------------------------- ;
	
		// ---- private auxiliary functions ------------------------------------- ;

	protected:

		// ---- protected data -------------------------------------------------- ;

			CRITICAL_SECTION m_csDevice;
			CStopWatch m_clTimer;

		// ---- protected auxiliary functions ----------------------------------- ;

			int getUnsignedLong(int iModuleId, unsigned long* puiData);

			int readChar(int iModuleId, int iCommandId, int iParameterId, char* pcData);
			int readUnsignedChar(int iModuleId, int iCommandId, int iParameterId, unsigned char* pucData);
			int readShort(int iModuleId, int iCommandId, int iParameterId, short* piData);
			int readUnsignedShort(int iModuleId, int iCommandId, int iParameterId, unsigned short* puiData);
			int readLong(int iModuleId, int iCommandId, int iParameterId, long* piData);
			int readUnsignedLong(int iModuleId, int iCommandId, int iParameterId, unsigned long* puiData);
			int readFloat(int iModuleId, int iCommandId, int iParameterId, float* pfData);

			int readLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long* piData, unsigned char* pucData1, unsigned char* pucData2);
			int readFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float* pfData, unsigned char* pucData1, unsigned char* pucData2);

			int write8Bytes(int iModuleId, bool bAck, void* pBytes); 

			int writeChar(int iModuleId, int iCommandId, int iParameterId, char cData);
			int writeUnsignedChar(int iModuleId, int iCommandId, int iParameterId, unsigned char ucData);
			int writeShort(int iModuleId, int iCommandId, int iParameterId, short iData);
			int writeUnsignedShort(int iModuleId, int iCommandId, int iParameterId, unsigned short uiData);
			int writeLong(int iModuleId, int iCommandId, int iParameterId, long iData);
			int writeUnsignedLong(int iModuleId, int iCommandId, int iParameterId, unsigned long uiData);
			int writeFloat(int iModuleId, int iCommandId, int iParameterId, float fData);

			int writeAll(int iCommandId, int iParameterId);
			int writeCommand(int iModuleId, int iCommandId);

			int writeLongShort(int iModuleId, int iCommandId, int iParameterId, long iData1, short iData2);
			int writeFloatShort(int iModuleId, int iCommandId, int iParameterId, float fData, short iData);

			int writeShortReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, short iData, long* piData, unsigned char* pucData1, unsigned char* pucData2);
			int writeLongReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long iData, long* piData, unsigned char* pucData1, unsigned char* pucData2);
			int writeLongShortReadLongUnsignedChars(int iModuleId, int iCommandId, int iParameterId, long iData1, short iData2, long* piData, unsigned char* pucData1, unsigned char* pucData2);
			int writeFloatReadFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float fData, float* pfData, unsigned char* pucData1, unsigned char* pucData2);
			int writeFloatShortReadFloatUnsignedChars(int iModuleId, int iCommandId, int iParameterId, float fData, short iData, float* pfData, unsigned char* pucData1, unsigned char* pucData2);

			virtual int reinit(unsigned char ucBaudRateId) = 0;
			virtual int clearReadQueue() = 0;
			virtual	int setMessageId(unsigned long uiMessageId) = 0;
			virtual	int readDevice(CProtocolMessage& rclProtocolMessage) = 0;
			virtual	int writeDevice(CProtocolMessage& rclProtocolMessage) = 0;
                        int printMessage(CProtocolMessage & rclProtocolMessage, bool read);

	public:

		// ---- public data ----------------------------------------------------- ;

	
		// ---- constructors / destructor --------------------------------------- ;

			/// default constructor
			CProtocolDevice();
			/// copy constructor
			CProtocolDevice(const CProtocolDevice& rclProtocolDevice);
			/// destructor
			virtual ~CProtocolDevice();

		// ---- operators ------------------------------------------------------- ;
		
			// assignment operator
			CProtocolDevice& operator=(const CProtocolDevice& rclProtocolDevice);

		// ---- query functions ------------------------------------------------- ;

			int getDataDLR_FTS(std::vector<float>& rafData, long* piState);
			int getDataSCHUNK_FTC(int iModuleId, int iChannelTypeId, std::vector<float>& rafData, short* piState);
			int getDataMP55_IO(int iModuleId, float* pfData);
			int getDataMP55_IO_fast(int iModuleId, float* pfData);
			int getCanOpenRawAbsEnc(int iModuleId, short* piValue);

		// ---- modify functions ------------------------------------------------ ;

			int setNullSCHUNK_FTC(int iModuleId, short* piState);
			int setTaraMP55_IO(int iModuleId, float fTara);
			int setInitMP55_IO_fast(int iModuleId);

		// ---- I/O functions --------------------------------------------------- ;

		// ---- exec functions -------------------------------------------------- ;

			int initDLR_FTS();

};

#endif
