
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


#include "Thread.h"

#if defined(__LINUX__)	
void* threadFunction(void* pvThread)
{
	CThread* pclThread = (CThread*)pvThread;
	(pclThread->m_pfuThreadFunction)(pclThread);
	return NULL;
}
#endif

#if defined(__QNX__)	
void threadFunction(void* pvThread)
{
	CThread* pclThread = (CThread*)pvThread;
	(pclThread->m_pfuThreadFunction)(pclThread);
}
#endif

#if defined(_WIN32)	
unsigned int __stdcall threadFunction(void* pvThread)
{
	CThread* pclThread = (CThread*)pvThread;
	(pclThread->m_pfuThreadFunction)(pclThread);
	return 0;
}
#endif

// ========================================================================== ;
//                                                                            ;
// ---- constructors / destructor ------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CThread::CThread(void) : CMessage("CThread", g_iDebugLevel, g_bDebug, g_bDebugFile),
						 m_uiStackSize(1228000), 
						 m_pcStack(0),
						 m_hThreadHandle(0),
						 m_bThreadRunFlag(false),
						 m_bThreadStopFlag(false),
						 m_pvThreadObject(0),
						 m_pfuThreadFunction(0)
{
}

CThread::CThread(const CThread& clThread)
{
	error(-1, "copy contructor : method should no be called!");
}

CThread::~CThread(void)
{
	debug(1, "destructed");
}

// ========================================================================== ;
//                                                                            ;
// ---- operators ----------------------------------------------------------- ;
//                                                                            ;
// ========================================================================== ;

CThread& CThread::operator=(const CThread& clThread)
{
	error(-1, "assignment operator : method should not be called!");
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

void CThread::setThreadStackSize(unsigned int uiSize)
{
	m_uiStackSize = uiSize;
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

int CThread::createThread(void (*pfuThreadFunction)(CThread*), void* pvThreadObject)
{
	m_bThreadStopFlag = false;
	m_pvThreadObject = pvThreadObject;
	m_pfuThreadFunction = pfuThreadFunction;
#if defined(_WIN32)					
	unsigned int iThreadId;

	m_hThreadHandle = (HANDLE)_beginthreadex(NULL, 0, threadFunction, (void*)this, 0, &iThreadId);

	if(m_hThreadHandle == NULL)
	{
		warning("createThread : creating thread failed!");
		m_bThreadRunFlag = false;
		return -1;
	}
	else
	{
		m_bThreadRunFlag = true;
		return 0;
	}
#endif
#if defined(__LINUX__)	
	pthread_attr_t Thread_attr;
        
    int retVal = pthread_create(&m_hThreadHandle, NULL, threadFunction, (void*)this);     
	if(retVal != 0)
 	{
 		warning("createThread : creating thread failed!");
		m_bThreadRunFlag = false; 
		return -1; 	
	}
	else
	{
		m_bThreadRunFlag = true;
		return 0;
	}
#endif
#if defined(__QNX__)

	if(m_pcStack == NULL)
		m_pcStack = new char[m_uiStackSize];

	if(m_pcStack == NULL)
	{
		m_bThreadRunFlag = false;
		warning("createThread : creating stack failed!");
		return -1;
	}
	int iThreadId = _beginthread(threadFunction, m_pcStack, m_uiStackSize, (void*)this);
	debug(1,"CThread: create stacksize=%d\n",m_uiStackSize);

	if( iThreadId == 0)
	{
		warning("createThread : creating thread failed!");
		m_bThreadRunFlag = false;
		delete [] m_pcStack;
		return -1;
	}
	else
	{
		m_bThreadRunFlag = true;
		return 0;
	}

#endif
}

void CThread::exitThread()
{
	m_bThreadRunFlag = false;
	#if defined(__WIN32)
		 _endthreadex(0);
	#endif
	#if defined(__QNX__)
		 _endthread();
	#endif
}

void CThread::terminateThread()
{
	m_bThreadStopFlag = true;
}

bool CThread::checkThreadRun()
{
	return m_bThreadRunFlag;
}

bool CThread::checkThreadStop()
{
	return m_bThreadStopFlag;
}
