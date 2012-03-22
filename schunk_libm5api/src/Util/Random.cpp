
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


#include "../Util/Random.h"

// -------------------------------------------------------------------------- ;

// \fdd{This function is an initialization entry point which should
//		be invoked before util\_random() is called although an
//		initializer value (FOR WIN32: derived from time()-function) is
//		supplied automatically.
//		With the same negative seed value a sequence of generated 
//		random values can be reproduced at any time.}

void util_seedRandom(long seedValA)
{
#if defined(_WIN32)
	fut_initRanG = seedValA;
#else
	srand48(seedValA);
#endif
};

// -------------------------------------------------------------------------- ;

// \fdd{This function returns non-negative
//		double-precision floating-point values uniformly distributed
//		over the interval $[0.0, 1.0)$ .}

double util_random()
{
#if defined(_WIN32)

	#define IA 16807
	#define IM 2147483647
	#define AM (1.0/IM)
	#define IQ 127773
	#define IR 2836
	#define NTAB 32
	#define NDIV (1+(IM-1)/NTAB)
	#define EPS 1.2e-7
	#define RNMX (1.0-EPS)

	int j;
	long k;
	static long iy=0;
	static long iv[NTAB];
	double temp;

	if (fut_initRanG <= 0 || !iy) {
		if (-(fut_initRanG) < 1) fut_initRanG=1;
		else fut_initRanG = -(fut_initRanG);
		for (j=NTAB+7;j>=0;j--) {
			k=(fut_initRanG)/IQ;
			fut_initRanG=IA*(fut_initRanG-k*IQ)-IR*k;
			if (fut_initRanG < 0) fut_initRanG += IM;
			if (j < NTAB) iv[j] = fut_initRanG;
		}
		iy=iv[0];
	}
	k=(fut_initRanG)/IQ;
	fut_initRanG=IA*(fut_initRanG-k*IQ)-IR*k;
	if (fut_initRanG < 0) fut_initRanG += IM;
	j=iy/NDIV;
	iy=iv[j];
	iv[j] = fut_initRanG;
	if ((temp=AM*iy) > RNMX) return RNMX;
	else return temp;
	#undef IA
	#undef IM
	#undef AM
	#undef IQ
	#undef IR
	#undef NTAB
	#undef NDIV
	#undef EPS
	#undef RNMX

#else
	return drand48();
#endif
};

// -------------------------------------------------------------------------- ;

// \fdd{This function returns
//		double-precision floating-point values gaussian distributed
//		with mean $0.0$ and variance $1.0$.  See \emph{Numerical Recipies} , 
//       page 289 for further references. }
double util_gaussRandom()
{
	static int isetL = 0;
	static double gsetL;
	double facL, rsqL, v1L, v2L;

	if( isetL == 0 )
	{
		do
		{
			v1L = 2.0 * util_random() - 1.0;
			v2L = 2.0 * util_random() - 1.0;
			rsqL = v1L * v1L + v2L * v2L;
		}
		while( rsqL >= 1.0 || rsqL == 0.0 );
		facL = sqrt( -2.0 * log( rsqL ) / rsqL );
		gsetL = v1L * facL;
		isetL = 1;
		return v2L * facL;
	}
	else
	{
		isetL = 0;
		return gsetL;
	}
}

// -------------------------------------------------------------------------- ;
