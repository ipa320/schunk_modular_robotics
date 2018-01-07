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
