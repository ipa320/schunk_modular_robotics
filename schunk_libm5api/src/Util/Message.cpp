
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


#include "Message.h"

int g_iDebugLevel = 3;
bool g_bDebugFile = false;
bool g_bDebug = true;
char* g_pcDebugFileName = "debug.txt";

double CMessage::m_fInitTime;

CRITICAL_SECTION* CMessage::m_csMessage = NULL;

#define ENTERCS if(m_csMessage!=NULL) EnterCriticalSection(m_csMessage);

#define LEAVECS if(m_csMessage!=NULL) LeaveCriticalSection(m_csMessage);

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CMessage::CMessage() : m_bDebug(g_bDebug), m_bDebugFile(g_bDebugFile), m_iDebugLevel(g_iDebugLevel)
{
	m_acClassName[0] = 0;
}

CMessage::CMessage(const char* pcClassName, int iDebuglevel, bool bDebug, bool bDebugFile) : m_bDebug(bDebug), m_bDebugFile(bDebugFile), m_iDebugLevel(iDebuglevel)
{
	strncpy(m_acClassName ,pcClassName, 50);
}

CMessage::CMessage(const CMessage& clMessage) : m_bDebug(clMessage.m_bDebug), m_bDebugFile(clMessage.m_bDebugFile), m_iDebugLevel(clMessage.m_iDebugLevel)
{
	strncpy(m_acClassName ,clMessage.m_acClassName, 50);
}

CMessage::~CMessage(void)
{
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CMessage& CMessage::operator=(const CMessage& clMessage)
{
	strncpy(m_acClassName ,clMessage.m_acClassName, 50);
	m_bDebug = clMessage.m_bDebug;
	m_bDebugFile = clMessage.m_bDebugFile;
	m_iDebugLevel = clMessage.m_iDebugLevel;
	return *this;
}

// ========================================================================== ;
//                                                                            ;
// ---- query functions ----------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

int CMessage::getDebugLevel() const
{
	return m_iDebugLevel;
}

// ========================================================================== ;
//                                                                            ;
// ---- modify functions ---------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

int CMessage::initMessage(const char* pcClassName, int iDebuglevel, bool bDebug, bool bDebugFile)
{
	strncpy(m_acClassName, pcClassName, 50);
	m_bDebug = bDebug;
	m_bDebugFile = bDebugFile;
	m_iDebugLevel = iDebuglevel;
	return 0;
}

void CMessage::setInitTime()
{
	#if defined(__QNX__)
		timespec nowTimeVal;
		clock_gettime(CLOCK_REALTIME,&nowTimeVal);
		m_fInitTime = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_nsec)/1e+9));
	#elif defined(_WIN32)
		_timeb nowTimeVal;
		_ftime(&nowTimeVal);
		m_fInitTime = (nowTimeVal.time
		  +(double(nowTimeVal.millitm)/1e+3));
	#else
		timeval nowTimeVal;
		gettimeofday(&nowTimeVal,0);
		m_fInitTime = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_usec)/1e+6));
	#endif
}

void CMessage::setDebugLevel(int iLevel)
{
	m_iDebugLevel = iLevel;
}

void CMessage::setDebug(bool bFlag)
{
	m_bDebug = bFlag;
}

void CMessage::setDebugFile(bool bFlag)
{
	m_bDebugFile = bFlag;
}

void CMessage::setCriticalSection(CRITICAL_SECTION *csMessage)
{
	m_csMessage = csMessage;
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

void CMessage::error(const char *pcErrorMessage,...) const
{

	ENTERCS;
	va_list args;
  
	va_start(args, pcErrorMessage);

	#if defined(__QNX__)
		timespec nowTimeVal;
		clock_gettime(CLOCK_REALTIME,&nowTimeVal);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_nsec)/1e+9)) - m_fInitTime;
	#elif defined(_WIN32)
		_timeb nowTimeVal;
		_ftime(&nowTimeVal);
		double fSeconds = (nowTimeVal.time
		  +(double(nowTimeVal.millitm)/1e+3)) - m_fInitTime;
	#else
		timeval nowTimeVal;
		gettimeofday(&nowTimeVal,0);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_usec)/1e+6)) - m_fInitTime;
	#endif

	static char acBuffer[255];	
	static char acOutBuffer[300];
	vsprintf(acBuffer, pcErrorMessage, args); 	
	sprintf(acOutBuffer, "\nERROR: %5.3f %s::%s", fSeconds, m_acClassName, acBuffer); 	
	if (m_bDebugFile==true)
	{	
		
		FILE* hFile;
		hFile=fopen(g_pcDebugFileName,"a+");
		if(hFile != NULL)
		{
			fprintf(hFile, "%s", acOutBuffer);
			fclose(hFile);
		}
	}
		
#ifdef WIN32
	OutputDebugString(acOutBuffer);
#else
	fprintf(stderr, acOutBuffer);
#endif

	va_end(args);
	LEAVECS;
	exit(-1);

};

void CMessage::error(const int iErrorCode,                      
                    const char *pcErrorMessage,...)const
{

	ENTERCS;
	va_list args;
  
	va_start(args, pcErrorMessage);

	#if defined(__QNX__)
		timespec nowTimeVal;
		clock_gettime(CLOCK_REALTIME,&nowTimeVal);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_nsec)/1e+9)) - m_fInitTime;
	#elif defined(_WIN32)
		_timeb nowTimeVal;
		_ftime(&nowTimeVal);
		double fSeconds = (nowTimeVal.time
		  +(double(nowTimeVal.millitm)/1e+3)) - m_fInitTime;
	#else
		timeval nowTimeVal;
		gettimeofday(&nowTimeVal,0);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_usec)/1e+6)) - m_fInitTime;
	#endif

	static char acBuffer[255];	
	static char acOutBuffer[300];
	vsprintf(acBuffer, pcErrorMessage, args); 	
	sprintf(acOutBuffer, "\nERROR: #%i %5.3f %s::%s", iErrorCode, fSeconds, m_acClassName, acBuffer); 	
	if (m_bDebugFile==true)
	{	
		
		FILE* hFile;
		hFile=fopen(g_pcDebugFileName,"a+");
		if(hFile != NULL)
		{
			fprintf(hFile, "%s", acOutBuffer);
			fclose(hFile);
		}
		
	}
		
#ifdef WIN32
	OutputDebugString(acOutBuffer);
#else
	fprintf(stderr, acOutBuffer);
#endif
	LEAVECS;
	exit(-1);

};

void CMessage::warning(const char *pcWarningMessage,...) const
{
        //UHR:use m_Debug as flag for screen output
	//if(!m_bDebug)
        //		return;
	ENTERCS;
	va_list args;
  
	va_start(args, pcWarningMessage);

	#if defined(__QNX__)
		timespec nowTimeVal;
		clock_gettime(CLOCK_REALTIME,&nowTimeVal);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_nsec)/1e+9)) - m_fInitTime;
	#elif defined(_WIN32)
		_timeb nowTimeVal;
		_ftime(&nowTimeVal);
		double fSeconds = (nowTimeVal.time
		  +(double(nowTimeVal.millitm)/1e+3)) - m_fInitTime;
	#else
		timeval nowTimeVal;
		gettimeofday(&nowTimeVal,0);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_usec)/1e+6)) - m_fInitTime;
	#endif

	static char acBuffer[255];	
	static char acOutBuffer[300];
	vsprintf(acBuffer, pcWarningMessage, args); 	
	sprintf(acOutBuffer, "\nWARNING: %5.3f %s::%s", fSeconds, m_acClassName, acBuffer); 	
	sprintf(acOutBuffer, "\nWARNING: %s::%s", m_acClassName, acBuffer); 	
	if (m_bDebugFile==true)
	{		
		FILE* hFile;
		hFile=fopen(g_pcDebugFileName,"a+");
		if(hFile != NULL)
		{
			fprintf(hFile, "%s", acOutBuffer);
			fclose(hFile);
		}
		
	}
		
#ifdef WIN32
	OutputDebugString(acOutBuffer);
#else
        if (m_bDebug)
	        fprintf(stderr, acOutBuffer);
#endif

	va_end(args);
	LEAVECS;

};

void CMessage::logging(const char *pcLoggingMessage,...) 
{

	ENTERCS;
	static char acBuffer[255];	
	va_list args;
	va_start(args, pcLoggingMessage);
	vsprintf(acBuffer, pcLoggingMessage, args); 	
	va_end(args);
	FILE *m_hLogFile=fopen("log.txt","a+");
	if(m_hLogFile != NULL)
	{
		fprintf(m_hLogFile,"%s",acBuffer);	
		fclose(m_hLogFile);
	}
	LEAVECS;

};

void CMessage::debug(const int iDebugLevel,
                      const char *pcDebugMessage,...) const
{

  //UHR:use m_Debug as flag for screen output
	//orig: if(iDebugLevel > m_iDebugLevel || !m_bDebug)
	if(iDebugLevel > m_iDebugLevel )
		return;
	ENTERCS;

	va_list args;
  
	va_start(args, pcDebugMessage);

	#if defined(__QNX__)
		timespec nowTimeVal;
		clock_gettime(CLOCK_REALTIME,&nowTimeVal);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_nsec)/1e+9)) - m_fInitTime;
	#elif defined(_WIN32)
		_timeb nowTimeVal;
		_ftime(&nowTimeVal);
		double fSeconds = (nowTimeVal.time
		  +(double(nowTimeVal.millitm)/1e+3)) - m_fInitTime;
	#else
		timeval nowTimeVal;
		gettimeofday(&nowTimeVal,0);
		double fSeconds = (nowTimeVal.tv_sec
		  +(double(nowTimeVal.tv_usec)/1e+6)) - m_fInitTime;
	#endif
	
	static char acBuffer[255];	
	static char acOutBuffer[300];
	vsprintf(acBuffer, pcDebugMessage, args); 	
	sprintf(acOutBuffer, "\nDEBUG: %i %5.3f %s::%s", iDebugLevel, fSeconds, m_acClassName, acBuffer); 	
	if (m_bDebugFile==true)
	{	
		
		FILE* hFile;
		hFile=fopen(g_pcDebugFileName,"a+");
		if(hFile != NULL)
		{
			fprintf(hFile, "%s", acOutBuffer);
			fclose(hFile);
		}
		
	}
		
#ifdef WIN32
	OutputDebugString(acOutBuffer);
#else
        if (m_bDebug)
        {
	        fprintf(stderr, acOutBuffer);
        }
#endif

	va_end(args);
	LEAVECS;

};
