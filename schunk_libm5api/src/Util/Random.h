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
