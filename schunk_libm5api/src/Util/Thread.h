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

#ifndef UTIL_THREAD_H
#define UTIL_THREAD_H

// ---- local includes ------------------------------------------------------ ;

#include "../Util/Message.h"

// ---- global includes ----------------------------------------------------- ;

// ---- constants ----------------------------------------------------------- ;

// ---- typedefs ------------------------------------------------------------ ;

// ---- class definition ---------------------------------------------------- ;

class CThread : public CMessage
{

private:

	// ---- private data ---------------------------------------------------- ;

	// ---- private auxiliary functions -------------------------------------- ;

protected:
	
	// ---- protected data -------------------------------------------------- ;

		unsigned int m_uiStackSize;
		char* m_pcStack;
#if defined (_WIN32)
		HANDLE m_hThreadHandle;
#endif
#if defined(__LINUX__)
		pthread_t m_hThreadHandle;
#endif
#if defined (__QNX__)
		void* m_hThreadHandle;
#endif
		bool m_bThreadRunFlag;
		bool m_bThreadStopFlag;

	// ---- protected auxiliary functions ------------------------------------- ;

public:

	// ---- public data ------------------------------------------------------- ;

		void* m_pvThreadObject;
		void (*m_pfuThreadFunction)(CThread*);

		// ---- constructors / destructor ----------------------------------------- ;

		// default constructor
		CThread();
		// copy constructor
		CThread(const CThread& clThread);
		// destructor
		~CThread(void);

	// ---- operators --------------------------------------------------------- ;
	
		// assignment operator
		CThread& operator=(const CThread& clThread);

	// ---- query functions --------------------------------------------------- ;

	// ---- modify functions -------------------------------------------------- ;
	
		void setThreadStackSize(unsigned int uiSize);

	// ---- I/O functions ----------------------------------------------------- ;

	// ---- exec functions ---------------------------------------------------- ;

		int createThread(void (*fuThreadFunction)(CThread*), void* pThreadObject);
		
		/// called inside the thread function before leaving the thread
		void exitThread(); 

		/// called outside the thread function to terminate the thread
		void terminateThread();

		/// check outside the thread function to check running
		bool checkThreadRun();

		/// check inside the thread function to check termination
		bool checkThreadStop();
};

#endif
