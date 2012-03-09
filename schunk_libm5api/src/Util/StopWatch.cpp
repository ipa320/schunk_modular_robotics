
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


#include "StopWatch.h"

// ========================================================================== ;
//                                                                            ;
//                  private auxiliary functions          
//                                                                            ;
// ========================================================================== ;

// ========================================================================== ;
//                                                                            ;
//                  protected auxiliary functions
//                                                                            ;
// ========================================================================== ;

// ========================================================================= ;
//                                                                           ;
//                      constructors / destructor                            ;
//                                                                           ;
// ========================================================================= ;

CStopWatch::CStopWatch()
	  : CMessage("CStopWatch", g_iDebugLevel, g_bDebug, g_bDebugFile),
		m_bStartFlag(false),
		m_bStopFlag(false),
		m_iTimeType(util_REAL_TIME)
{
		m_iFirst = 0;
		m_iLast = 0;
		m_fOverflowTime = 0.0;
		
#if defined(_WIN32)
		QueryPerformanceFrequency(&m_TempTime);   // pointer to current frequency
		frequencyE = m_TempTime.LowPart;
		m_FirstTime.HighPart = 0;
		m_FirstTime.LowPart = 0;
		m_LastTime.HighPart = 0;
		m_LastTime.LowPart = 0;
		m_TempTime.HighPart = 0;
		m_TempTime.LowPart = 0;
		m_ActualTime.HighPart = 0;
		m_ActualTime.LowPart = 0;
#elif defined(__QNX__)
		m_FirstTime.tv_sec = 0;
		m_FirstTime.tv_nsec = 0;
		m_LastTime.tv_sec = 0;
		m_LastTime.tv_nsec = 0;
		m_TempTime.tv_sec = 0;
		m_TempTime.tv_nsec = 500000;
		clock_setres(CLOCK_REALTIME, &m_TempTime);
		m_ActualTime.tv_sec = 0;
		m_ActualTime.tv_nsec = 0;
#else
		m_FirstTime.tv_sec = 0;
		m_FirstTime.tv_usec = 0;
		m_LastTime.tv_sec = 0;
		m_LastTime.tv_usec = 0;
		m_TempTime.tv_sec = 0;
		m_TempTime.tv_usec = 0;
		m_ActualTime.tv_sec = 0;
		m_ActualTime.tv_usec = 0;
#endif
}

CStopWatch::CStopWatch(util_TimeMeasurementType iTimeType)
	  : CMessage("CStopWatch", g_iDebugLevel, g_bDebug, g_bDebugFile),
		m_bStartFlag(false),
		m_bStopFlag(false),
		m_iTimeType(iTimeType)
{
		m_iFirst = 0;
		m_iLast = 0;
		m_fOverflowTime = 0.0;
		
#if defined(_WIN32)
		QueryPerformanceFrequency(&m_TempTime);   // pointer to current frequency
		frequencyE = m_TempTime.LowPart;
		m_FirstTime.HighPart = 0;
		m_FirstTime.LowPart = 0;
		m_LastTime.HighPart = 0;
		m_LastTime.LowPart = 0;
		m_TempTime.HighPart = 0;
		m_TempTime.LowPart = 0;
		m_ActualTime.HighPart = 0;
		m_ActualTime.LowPart = 0;
#elif defined(__QNX__)
		m_FirstTime.tv_sec = 0;
		m_FirstTime.tv_nsec = 0;
		m_LastTime.tv_sec = 0;
		m_LastTime.tv_nsec = 0;
		m_TempTime.tv_sec = 0;
		m_TempTime.tv_nsec = 500000;
		clock_setres(CLOCK_REALTIME, &m_TempTime);
		m_ActualTime.tv_sec = 0;
		m_ActualTime.tv_nsec = 0;
#else
		m_FirstTime.tv_sec = 0;
		m_FirstTime.tv_usec = 0;
		m_LastTime.tv_sec = 0;
		m_LastTime.tv_usec = 0;
		m_TempTime.tv_sec = 0;
		m_TempTime.tv_usec = 0;
		m_ActualTime.tv_sec = 0;
		m_ActualTime.tv_usec = 0;
#endif
}

CStopWatch::CStopWatch(const CStopWatch& )
{
	error(-1, "copy constructor : method should not be called!");
}

// ========================================================================= ;
//                                                                           ;
//                  operators
//                                                                           ;
// ========================================================================= ;

CStopWatch& CStopWatch::operator=(const CStopWatch&)
{
	error(-1, "assignment operator : method should not be called!");
	return *this;
}

// ========================================================================= ;
//                                                                           ;
//                  query functions
//                                                                           ;
// ========================================================================= ;

// ========================================================================= ;
//                                                                           ;
//                  modifiy functions
//                                                                           ;
// ========================================================================= ;

// ========================================================================= ;
//                                                                           ;
//                  I/O
//                                                                           ;
// ========================================================================= ;

/*
	This member function computes the difference between the taken start and stop times, if both are valid.
	Attention: cpu-time is calculated in a wrong way, if process lasts longer than 35 min or 2 processes
	            are stoped with the second beginning more than half an hour after the first one was started.
	           Look for function testOverflow in any of these cases.
	 difference or $0.0$ if call was invalid
*/
double CStopWatch::executionTime()
{
	// --- check
	if ( !(m_bStartFlag && m_bStopFlag) )
	{
		warning("executionTime() : return 0.0, for you must call 'start()' and 'stop()' first");
		return 0.0;
	};

	if(m_iTimeType == util_CPU_TIME)
	{
		if ((m_iLast < m_iFirst) && (m_fOverflowTime == 0))
		{
			warning("executionTime() : return 0.0, for start time is bigger than stop time and no overflow was detected");
			return 0.0;
		}
		else
		{
			double fTempTime;
			testOverflow();
			fTempTime = m_fOverflowTime;
			m_fOverflowTime = 0.0;
			return fTempTime + (double(m_iLast - m_iFirst)) / CLOCKS_PER_SEC;
		};
	}
	else
	{
		#if defined(__QNX__)
		return (m_LastTime.tv_sec-m_FirstTime.tv_sec
		  +(double(m_LastTime.tv_nsec-m_FirstTime.tv_nsec)/1e+9));
		#elif defined(_WIN32)
		return double(m_LastTime.LowPart-m_FirstTime.LowPart)/frequencyE;
		#else
		return (m_LastTime.tv_sec-m_FirstTime.tv_sec
		  +(double(m_LastTime.tv_usec-m_FirstTime.tv_usec)/1e+6));
		#endif	 
	};
};

// ========================================================================= ;
//                                                                           ;
//                  exec functions
//                                                                           ;
// ========================================================================= ;

void  CStopWatch::start() 
{
	if(m_iTimeType == util_CPU_TIME)
	{
		m_iFirst=clock();
		m_bStartFlag = true;
		m_bStopFlag = false;
	}
	else
	{
#if defined(__QNX__)
		clock_gettime(CLOCK_REALTIME,&m_FirstTime);
#elif defined(_WIN32)
		QueryPerformanceCounter(&m_FirstTime);
#else
		gettimeofday(&m_FirstTime,0);
#endif
		m_bStartFlag = true;
		m_bStopFlag = false;
	};
};

/*
	This member function sets the stop time of the watch to the current clock value, if a valid start time exists.
*/
void CStopWatch::stop()
{
	if (m_bStartFlag)
	{
		if(m_iTimeType == util_CPU_TIME)
		{
			m_iLast = clock();
		}
		else
		{
			#if defined(__QNX__)
			clock_gettime(CLOCK_REALTIME,&m_LastTime);
			#elif defined(_WIN32)
			QueryPerformanceCounter(&m_LastTime);
			#else
			gettimeofday(&m_LastTime,0);
			#endif
		};
		m_bStopFlag = true;
	}
	else
	{
		m_bStopFlag = false;
		warning("stop() : you must call 'start()' first");
	};
};

// -------------------------------------------------------------------------- ;

/*
	This member function continues the run of the clock after is has been stopped.
*/
void CStopWatch::cont()
{
	if (m_bStartFlag && m_bStopFlag)
	{
		if(m_iTimeType == util_CPU_TIME)
		{  
			m_iFirst      = m_iFirst + (clock() - m_iLast);
			m_bStopFlag  = false;
		}
		else
		{
			#if defined(__QNX__)
			clock_gettime(CLOCK_REALTIME,&m_TempTime);
			m_FirstTime.tv_sec +=  m_TempTime.tv_sec-m_LastTime.tv_sec;
			m_FirstTime.tv_nsec +=  m_TempTime.tv_nsec-m_LastTime.tv_nsec;
			#elif defined(_WIN32)
			QueryPerformanceCounter(&m_TempTime);
			m_FirstTime.HighPart +=  m_TempTime.HighPart-m_LastTime.HighPart;
			m_FirstTime.LowPart +=  m_TempTime.LowPart-m_LastTime.LowPart;
			#else
			gettimeofday(&m_TempTime,0);
			m_FirstTime.tv_sec +=  m_TempTime.tv_sec-m_LastTime.tv_sec;
			m_FirstTime.tv_usec +=  m_TempTime.tv_usec-m_LastTime.tv_usec;
			#endif
		};
	}
	else
		warning("cont() : you must call 'start()' and 'stop()' first");
};

// This member function returns the current real time in [s].
double CStopWatch::realTime() 
{
	#if defined(__QNX__)
	clock_gettime(CLOCK_REALTIME,&m_ActualTime);
	return (m_ActualTime.tv_sec+(double(m_ActualTime.tv_nsec)/1e+9));
	#elif defined(_WIN32)
	QueryPerformanceCounter(&m_ActualTime);
	return (double(m_ActualTime.LowPart)/frequencyE);
	#else
	gettimeofday(&m_ActualTime,0);
	return (m_ActualTime.tv_sec
	  +(double(m_ActualTime.tv_usec)/1e+6));
	#endif
};

// This member function returns the real time resolution in [s].
double CStopWatch::realTimeResolution() 
{
	#if defined(__QNX__)
	clock_getres(CLOCK_REALTIME,&m_TempTime);
	return (m_TempTime.tv_sec+(double(m_TempTime.tv_nsec)/1e+9));
	#elif defined(_WIN32)
	warning("unkown real time resolution\n");
	return 0.001;
	#else
	warning("unkown real time resolution\n");
	return 0.001;
	#endif
};

/*
	This member function waits the given realtime in [ms].
	uiTime: realtime in [ms]
*/
void CStopWatch::wait(unsigned int uiTime)
{
	bool bTimeOutFlag = false;
#if defined(__QNX__)
	unsigned int uiSec, uiNSec;
	uiSec = uiTime / 1000;
	uiNSec = (uiTime % 1000) * 1000000;
	clock_gettime(CLOCK_REALTIME, &m_TempTime);
	m_TempTime.tv_sec = m_TempTime.tv_sec + uiSec + (m_TempTime.tv_nsec + uiNSec) / 1000000000;
	m_TempTime.tv_nsec = (m_FirstTime.tv_nsec + uiNSec) % 1000000000;
	do
	{
		clock_gettime(CLOCK_REALTIME, &m_ActualTime);
		if(m_ActualTime.tv_sec > m_TempTime.tv_sec)
			bTimeOutFlag = true;
		else if((m_ActualTime.tv_sec == m_TempTime.tv_sec) && (m_ActualTime.tv_nsec > m_TempTime.tv_nsec))
			bTimeOutFlag = true;
	}
	while(!bTimeOutFlag);
#elif defined(_WIN32)
	Sleep(uiTime);
#else
	unsigned int uiSec, uiUSec;
	uiSec = uiTime / 1000;
	uiUSec = (uiTime % 1000) * 1000;
	gettimeofday(&m_TempTime, 0);
	m_TempTime.tv_sec = m_TempTime.tv_sec + uiSec + (m_TempTime.tv_usec + uiUSec) / 1000000;
	m_TempTime.tv_usec = (m_TempTime.tv_usec + uiUSec) % 1000000;
	do
	{
		gettimeofday(&m_ActualTime, 0);
		if(m_ActualTime.tv_sec > m_TempTime.tv_sec)
			bTimeOutFlag = true;
		else if((m_ActualTime.tv_sec == m_TempTime.tv_sec) && (m_ActualTime.tv_usec > m_TempTime.tv_usec))
			bTimeOutFlag = true;
	}
	while(!bTimeOutFlag);
#endif
};

// -------------------------------------------------------------------------- ;

/*
	This member function returns the current date and time in local representation
	char* to hold the local date and time
*/
void CStopWatch::date(char* acDate)  const
{
	time_t currentTime;
	struct tm *localTime;

	// get the current time
	currentTime = time (NULL);

	// convert it to local time representation
	localTime = localtime (&currentTime);

	// print it out in a buffer
	strftime (acDate, 256, "%a %b %d %I:%M:%S %p %Z %Y", localTime);
};

// -------------------------------------------------------------------------- ;

/*
	This member function returns the current weekday in local representation
	string to hold the local weekday
*/
void CStopWatch::weekday(char* acWeekDay)  const
{
	time_t currentTime;
	struct tm *localTime;

	// get the current time
	currentTime = time (NULL);

	// convert it to local time representation
	localTime = localtime (&currentTime);

	// print it out in a buffer
	strftime (acWeekDay, 256, "%A", localTime);
};

// ========================================================================= ;

/*
	This function tests, if there is an overflow at the clock counter
	Function should be called at least every 30 min., if the cpu_time is stoped
	Otherwise there might be more than one overflow and results are wrong!
*/
void CStopWatch::testOverflow()
{
	if(m_iTimeType == util_CPU_TIME)
	{
		stop();
		if (m_iLast < m_iFirst)
		{
			m_fOverflowTime += (double(m_iLast - m_iFirst + ULONG_MAX)) / CLOCKS_PER_SEC;
			start();
		}
		else
			cont();
	}
	else
		warning("testOverflow() : overflow has to be tested only when measuring cpu-time");
};

// ========================================================================= ;
