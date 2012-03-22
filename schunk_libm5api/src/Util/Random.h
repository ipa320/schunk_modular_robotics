
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


#ifndef UTIL_RANDOM_H
#define UTIL_RANDOM_H

// ---- local includes ------------------------------------------------------ ;

#include "../Util/GlobalDefines.h"

// ---- global includes ----------------------------------------------------- ;

#if defined(_WIN32)
	#include <stdio.h>
	#include "Math.h"
	#include <stdlib.h>
	#include "Values.h"
	#include <time.h>
	static long fut_initRanG = -time((time_t *)0);
#elif defined(__QNX__)
	#include <stdio.h>
	#include "Math.h"
	#include <stdlib.h>
	#include "Values.h"
	#include <unix.h>		
#else
//			extern "C" {
	#include <stdio.h>
	#include "Math.h"
	#include <stdlib.h>
	#include "Values.h"
//			}

	extern "C" double drand48(void);
	extern "C" double erand48(unsigned short xsubi[3]);
	extern "C" long lrand48(void);
	extern "C" long nrand48(unsigned short xsubi[3]);
	extern "C" long mrand48(void);
	extern "C" long jrand48(unsigned short xsubi[3]);
	extern "C" void srand48(long seedval);
	extern "C" unsigned short *seed48(unsigned short seed16v[3]);
	extern "C" void lcong48(unsigned short param[7]);
#endif

// ---- constants ----------------------------------------------------------- ;

// ---- typedefs ------------------------------------------------------------ ;

// ---- external functions -------------------------------------------------- ;

// ---- class definition ---------------------------------------------------- ;

// ---- extern I/O ---------------------------------------------------------- ;

// ---- local functions ---------------------------------------------------- ;

// \fsd{sets an entry point for random number generators}

void util_seedRandom(long seedValA);

// \fsd{generate uniformly distributed pseudo-random numbers}

double util_random();

// \fsd{generate gaussian distributed pseudo-random numbers}

double util_gaussRandom();

// -------------------------------------------------------------------------- ;

#endif // UTIL_RANDOM_H
