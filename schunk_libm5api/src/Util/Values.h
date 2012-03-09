
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


#ifndef UTIL_VALUES_H
#define UTIL_VALUES_H

// ---- local includes ------------------------------------------------------ ;

// ---- global includes ----------------------------------------------------- ;

#if defined(__QNX__) || defined(_WIN32)

#ifdef _WIN32
#include <winsock2.h>    // prevent include of winsock.h inside windows.h!!
#include <windows.h>
#endif

#include <limits.h>
#include <float.h>

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * These values work with any binary representation of integers
 * where the high-order bit contains the sign.
 */

/* a number used normally for size of a shift */
#define	BITSPERBYTE	8

#define	BITS(type)	(BITSPERBYTE * (int)sizeof (type))

/* short, regular and long ints with only the high-order bit turned on */
#define	HIBITS	((short)(1 << BITS(short) - 1))

#if defined(__STDC__)

#define	HIBITI	(1U << BITS(int) - 1)
#define	HIBITL	(1UL << BITS(long) - 1)

#else

#define	HIBITI	((unsigned)1 << BITS(int) - 1)
#define	HIBITL	(1L << BITS(long) - 1)

#endif


#undef MAXINT
#define	MAXINT	  INT_MAX

#ifndef _WIN32_WINNT

#undef MAXSHORT
#undef MAXLONG

#define	MAXSHORT  SHRT_MAX    /* see <limits.h> */
#define	MAXLONG	  LONG_MAX

#endif

/*
 * various values that describe the binary floating-point representation
 * _EXPBASE	- the exponent base
 * DMAXEXP	- the maximum exponent of a double (as returned by frexp())
 * FMAXEXP	- the maximum exponent of a float  (as returned by frexp())
 * DMINEXP	- the minimum exponent of a double (as returned by frexp())
 * FMINEXP	- the minimum exponent of a float  (as returned by frexp())
 * MAXDOUBLE	- the largest double
 *			((_EXPBASE ** DMAXEXP) * (1 - (_EXPBASE ** -DSIGNIF)))
 * MAXFLOAT	- the largest float
 *			((_EXPBASE ** FMAXEXP) * (1 - (_EXPBASE ** -FSIGNIF)))
 * MINDOUBLE	- the smallest double (_EXPBASE ** (DMINEXP - 1))
 * MINFLOAT	- the smallest float (_EXPBASE ** (FMINEXP - 1))
 * DSIGNIF	- the number of significant bits in a double
 * FSIGNIF	- the number of significant bits in a float
 * DMAXPOWTWO	- the largest power of two exactly representable as a double
 * FMAXPOWTWO	- the largest power of two exactly representable as a float
 * _IEEE	- 1 if IEEE standard representation is used
 * _DEXPLEN	- the number of bits for the exponent of a double
 * _FEXPLEN	- the number of bits for the exponent of a float
 * _HIDDENBIT	- 1 if high-significance bit of mantissa is implicit
 * LN_MAXDOUBLE	- the natural log of the largest double  -- log(MAXDOUBLE)
 * LN_MINDOUBLE	- the natural log of the smallest double -- log(MINDOUBLE)
 * LN_MAXFLOAT	- the natural log of the largest float  -- log(MAXFLOAT)
 * LN_MINFLOAT	- the natural log of the smallest float -- log(MINFLOAT)
 */

#undef MAXDOUBLE
#undef MINDOUBLE
#undef MAXFLOAT
#undef MINFLOAT

#define	MAXDOUBLE	DBL_MAX     /* see <float.h> */
#define	MINDOUBLE	DBL_MIN
#define	MAXFLOAT	FLT_MAX
#define	MINFLOAT	FLT_MIN
#define	_IEEE		1
#define	_DEXPLEN	11
#define	_HIDDENBIT	1
#define	_LENBASE	1
#define	DMINEXP	(-(DMAXEXP + DSIGNIF - _HIDDENBIT - 3))
#define	FMINEXP	(-(FMAXEXP + FSIGNIF - _HIDDENBIT - 3))

#define	_EXPBASE	(1 << _LENBASE)
#define	_FEXPLEN	8
#define	DSIGNIF	(BITS(double) - _DEXPLEN + _HIDDENBIT - 1)
#define	FSIGNIF	(BITS(float)  - _FEXPLEN + _HIDDENBIT - 1)
#define	DMAXPOWTWO	((double)(1L << BITS(long) - 2) * \
				(1L << DSIGNIF - BITS(long) + 1))
#define	FMAXPOWTWO	((float)(1L << FSIGNIF - 1))
#define	DMAXEXP	((1 << _DEXPLEN - 1) - 1 + _IEEE)
#define	FMAXEXP	((1 << _FEXPLEN - 1) - 1 + _IEEE)
#define	LN_MAXDOUBLE	(M_LN2 * DMAXEXP)
#define	LN_MAXFLOAT	(float)(M_LN2 * FMAXEXP)
#define	LN_MINDOUBLE	(M_LN2 * (DMINEXP - 1))
#define	LN_MINFLOAT	(float)(M_LN2 * (FMINEXP - 1))
#define	H_PREC	(DSIGNIF % 2 ? (1L << DSIGNIF/2) * M_SQRT2 : 1L << DSIGNIF/2)
#define	FH_PREC \
	(float)(FSIGNIF % 2 ? (1L << FSIGNIF/2) * M_SQRT2 : 1L << FSIGNIF/2)
#define	X_EPS	(1.0/H_PREC)
#define	FX_EPS	(float)((float)1.0/FH_PREC)
#define	X_PLOSS	((double)(long)(M_PI * H_PREC))
#define	FX_PLOSS ((float)(long)(M_PI * FH_PREC))
#define	X_TLOSS	(M_PI * DMAXPOWTWO)
#define	FX_TLOSS (float)(M_PI * FMAXPOWTWO)
#define	M_LN2	0.69314718055994530942
#define	M_PI	3.14159265358979323846
#define	M_SQRT2	1.41421356237309504880
#define	MAXBEXP	DMAXEXP /* for backward compatibility */
#define	MINBEXP	DMINEXP /* for backward compatibility */
#define	MAXPOWTWO	DMAXPOWTWO /* for backward compatibility */

#ifdef	__cplusplus
}
#endif

#else
#include <values.h>
#endif

#endif // UTIL_VALUES_H
