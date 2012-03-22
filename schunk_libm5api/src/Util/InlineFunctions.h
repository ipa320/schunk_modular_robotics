
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


#ifndef UTIL_INLINEFUNCTIONS_H
#define UTIL_INLINEFUNCTIONS_H

#include "../Util/GlobalDefines.h"

#ifndef _WIN32
#include <unistd.h>
#endif

#include <time.h>
#include <limits.h>

#if defined (_WIN32)
#include <sys/timeb.h>
#include <windows.h>
#include <process.h>
#endif
#ifdef __QNX__
#include <sys/time.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <i86.h>
#include <process.h>
#define TRACE printf
#define CRITICAL_SECTION int
#endif

#ifdef __LINUX__
#include <sys/time.h>
//#include <linux/delay.h>
#include <pthread.h>
#define TRACE printf
#define CRITICAL_SECTION pthread_mutex_t
#endif
#include "../Util/Math.h"

//  returns the squared fValue
template <class T> inline T sqr(T fValue)     
{ 
	return fValue*fValue; 
};

// returns the rounded integer fValue
template <class T> inline int iRound(T v)      
{
	return (v>=0) ? (int)(v+.5) : (int)(v-.5);
};

// returns the minimum of fValue a and fValue b 
template <class T> inline T util_min(T a, T b)    
{ 
	return (a<b) ? a : b; 
};

// returns the maximum of fValue a and fValue b 
template <class T> inline T util_max(T a, T b)    
{ 
	return (a>b) ? a : b; 
};

#ifndef NO_ABS_FCT

// returns the absolute fValue
inline long abs(long iValue) 
{ 
#if defined(NO_CAST_FUNCTION_TEMPLATES)
	return long(abs(iValue));
#else
	return static_cast<long>(abs(iValue));
#endif
};

// returns the absolute uiValue
inline unsigned long abs(unsigned long uiValue) 
{ 
#if defined(NO_CAST_FUNCTION_TEMPLATES)
	return unsigned long(abs(uiValue));
#else
	return static_cast<unsigned long>(abs(uiValue));
#endif
};

// returns the absolute fValue 
inline float abs(float fValue) 
{ 
#if defined(NO_CAST_FUNCTION_TEMPLATES)
	return float(fabs(fValue));
#else
	return static_cast<float>(fabs(fValue));
#endif
};

// returns the absolute fValue
inline double abs(double fValue) 
{ 
	return fabs(fValue);
};

#endif

// returns fValue a with the sign of fValue b 
inline float util_sign(float a, float b) 
{ 
	return ((b) >= 0.0) ? fabs(a) : -fabs(a); 
};

// returns fValue a with the sign of fValue b 
inline double util_sign(double a, double b) 
{ 
	return ((b) >= 0.0) ? fabs(a) : -fabs(a); 
};

template <class T> inline void util_shift(T a, T b, T c, T d)
{
	(a)=(b);
	(b)=(c);
	(c)=(d);
}

// converts degrees to radians 
inline double util_degToRad(double fAngle)
{ 
	return fAngle * M_PI / 180.0; 
};

// converts radians to degrees 
inline double util_radToDeg(double fAngle)
{ 
	return fAngle * 180.0 / M_PI; 
};

// fits fPhase into the interval [0,2 \pi[
inline double util_adjustedPhase(double fPhase) 
{
	return fPhase - (2*M_PI)*floor(fPhase*M_1_2PI);
}

// computes fPhase1 - fPhase2 as a fValue of [-pi,pi[
inline double util_phaseDifference(double fPhase1, double fPhase2) 
{
	return util_adjustedPhase(fPhase1 - fPhase2 + M_PI) - M_PI;
}

// computes the average of fPhase1 and fPhase2 as a fValue of [0, 2pi[
inline double util_averagedPhase(double fPhase1, double fPhase2) 
{
	return util_adjustedPhase(fPhase1 + (util_phaseDifference(fPhase2,fPhase1)*0.5));
}

// exhanges the contents of two variables
template <class Type>
inline void util_swap(Type& a, Type& b) 
{
	Type swappy = a;
	a = b; b = swappy;
}

#if defined _WIN32

#ifndef __HAS_SLEEP__
#define __HAS_SLEEP__

// encapsulates the Win32 version of sleep called Sleep
inline void sleep(unsigned int uiSec)
{
#if defined(NO_CAST_FUNCTION_TEMPLATES)
	Sleep(DWORD(uiSec*1000));
#else
	Sleep(static_cast<DWORD> (uiSec*1000));
#endif
}
#endif
#endif

#if defined (__LINUX__)
inline int EnterCriticalSection(CRITICAL_SECTION *cs)
{
	pthread_mutex_lock(cs);
 	return 0;
}

inline int LeaveCriticalSection(CRITICAL_SECTION *cs)
{
 	pthread_mutex_unlock(cs);
	return 0;
}

inline int InitializeCriticalSection(CRITICAL_SECTION *cs)
{
	pthread_mutex_init(cs,NULL);
 	pthread_mutex_unlock(cs);
 	return 0;
}

inline int DeleteCriticalSection(CRITICAL_SECTION *cs)
{
//	pthread_mutex_exit(cs);
	return 0;
}

inline int Sleep(long iMilliSec)
{
    timespec tm, tm2;

	tm.tv_sec=iMilliSec/1000;
	tm.tv_nsec=(iMilliSec%1000)*1000000;
	
	nanosleep(&tm,&tm2);
 	return 0;
}
#endif

#if defined (__QNX__)
inline int EnterCriticalSection(CRITICAL_SECTION *cs)
{
	sem_wait( (sem_t *)cs );
 	return 0;
}
inline int LeaveCriticalSection(CRITICAL_SECTION *cs)
{
	sem_post( (sem_t *)cs );
 	return 0;
}
inline int InitializeCriticalSection(CRITICAL_SECTION *cs)
{
	sem_init( (sem_t*)cs, 1, 1 );
 	return 0;
}

inline int DeleteCriticalSection(CRITICAL_SECTION *cs)
{
//	sem_exit((sem_t*) cs);
	sem_destroy( (sem_t*)cs );
 	return 0;
}

inline int Sleep(long iMilliSec)
{
	delay(iMilliSec);
 	return 0;
}
#endif

// -------------------------------------------------------------------------- ;

// sets the alarm clock to the specified number of uiSec. 
/*
	sets the alarm clock to the specified number of uiSec.
	  NOTE for UNIX-systems: see the manual pages for alarm(2)
	  NOTE for WIN32-systems: does nothing! (just returns 0).
	uiSec: number of uiSec
	the amount of time  previously remaining in the alarm clock.
*/
inline unsigned int util_setAlarm(unsigned int uiSec)
{
#ifdef _WIN32
// there does not exist any alarm function for WIN32!
	return 0;
#else
	return alarm(uiSec);
#endif
};

// -------------------------------------------------------------------------- ;

// cancels any previously made alarm request.
/*
	cancels any previously made alarm request.
	  NOTE for UNIX-systems: see the manual pages for alarm(2)
	  NOTE for WIN32-systems: does nothing! (just returns 0).
	the amount of time  previously remaining in the alarm clock.
*/
inline unsigned int util_deactivateAlarm()
{
#ifdef _WIN32
// there does not exist any alarm function for WIN32!
	return 0;
#else
	return alarm(0);   // if number of uiSec is equal 0, any previously
					   // made alarm request is canceled
#endif
};

#endif // UTIL_INLINEFUNCTIONS_H
