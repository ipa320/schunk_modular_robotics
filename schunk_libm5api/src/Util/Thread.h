
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
