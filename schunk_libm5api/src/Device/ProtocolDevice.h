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
