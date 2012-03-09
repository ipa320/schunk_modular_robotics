
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
