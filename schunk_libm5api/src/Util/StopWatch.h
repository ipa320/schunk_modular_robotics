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

#ifndef UTIL_STOPWATCH_H
#define UTIL_STOPWATCH_H

// ---- local includes ------------------------------------------------------ ;

#include "../Util/Message.h"

// ---- global includes ----------------------------------------------------- ;

// ---- remap, typedefs ----------------------------------------------------- ;

enum util_TimeMeasurementType{
	util_CPU_TIME,
	util_REAL_TIME
};

// ---- class definition ---------------------------------------------------- ;

/*
	stopwatch features
	This class allows to measure discrete time intervalls in your program.
*/
class CStopWatch : public CMessage
{

	protected:

	// ---- protected datas  ------------------------------------------------ ;

		long m_iFirst;
		long m_iLast;
		double m_fOverflowTime;

		#if defined(__QNX__)
		timespec m_FirstTime;
		timespec m_LastTime;
		timespec m_TempTime;
		timespec m_ActualTime;
		#elif defined(_WIN32)
		double	frequencyE;
		LARGE_INTEGER m_FirstTime;
		LARGE_INTEGER m_LastTime;
		LARGE_INTEGER m_TempTime;
		LARGE_INTEGER m_ActualTime;
		#else
		timeval	m_FirstTime;
		timeval	m_LastTime;
		timeval	m_TempTime;
		timeval	m_ActualTime;
		#endif
 
		bool m_bStartFlag;
		bool m_bStopFlag;
  
		util_TimeMeasurementType m_iTimeType;
  
	// ---- auxiliary functions ----------------------------------------------- ;

	public:

	// ---- public datas ------------------------------------------------------ ;

	// ---- constructor/destructor -------------------------------------------- ;

		// default constructor with timetype util_REAL_TIME
		CStopWatch();
	
	
		// constructor
		CStopWatch(util_TimeMeasurementType iTimeType);

		// copy constructor
		CStopWatch(const CStopWatch& );

	// ---- operators --------------------------------------------------------- ;

		// assignment operator
		CStopWatch& operator=(const CStopWatch& );

	// ---- query functions --------------------------------------------------- ;

		// returns the time type
		util_TimeMeasurementType timeType() const;
 
	// ---- modify functions -------------------------------------------------- ;

		//  sets the time type
		void timeType(const util_TimeMeasurementType& riTimeType);
   
	// ---- I/O --------------------------------------------------------------- ;

		// returns the difference between start and stop time in [s]
		double executionTime();

		// returns the real time in [s]
		double realTime();
	
		// returns the real time resolution in [s]
		double realTimeResolution();

	// ---- exec functions ---------------------------------------------------- ;

		// sets start time to current clock value
		void start();
	
		// sets stop time to current clock value
		void stop();
  
		// continues after a call of stop()
		void cont();

		// wait realtime in [ms]
		void wait(unsigned int uiTime);
 	
		// returns the current date
		void date(char* acDate) const;
 	
		// returns the current weekday
		void weekday(char* acWeekday) const;
  
		// tests, if there is an overflow at clock counter
		void testOverflow();
};

// ---- extern I/O ---------------------------------------------------------- ;

// ---- public inline functions --------------------------------------------- ;

inline util_TimeMeasurementType CStopWatch::timeType() const
{ 
	return(m_iTimeType);
};

inline void CStopWatch::timeType(const util_TimeMeasurementType& riTimeType) 
{
	m_iTimeType = riTimeType;
};

// -------------------------------------------------------------------------- ;

#endif // UTIL_STOPWATCH_H
