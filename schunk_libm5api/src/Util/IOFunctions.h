
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


#ifndef UTIL_IOFUNCTIONS_H
#define UTIL_IOFUNCTIONS_H

// ---- global includes ----------------------------------------------------- ;

#include "../Util/GlobalDefines.h"
#include <stdio.h>
#include <stdlib.h>     // to use getenv
#include <iostream>

#if defined(__QNX__) && defined(__WATCOMC__)
#include <iomanip>
#endif

// ---- constants ----------------------------------------------------------- ;

const int BUFFER_LENGTH        = 256;

const int OKAY                 = 0;
const int KEY_BUT_NO_EQUAL     = 1;
const int NO_KEY               = 2;
const int FOUND_EOF            = 3;
const int NO_OPEN_BRACKET      = 4;
const int NO_SEPERATOR         = 5;
const int NO_CLOSED_BRACKET    = 6;
const int KEY_BUT_WRONG_NUMBER = 7;

// ---- local includes ------------------------------------------------------ ;

// ---- local functions ----------------------------------------------------- ;

int util_searchString(const char* acSectionName, const char* acKeyName, const char* acDefaultString, char* acReturnString, int iSize, const char* acFileName);

int util_setString(const char* acSectionName, const char* acKeyName, const char* acString, const char* acFileName);

#ifdef WITHSTREAMS
// checks for a given keyword and positions the stream at the
//		corresponding argument
int  util_posArgForKey(
				std::istream&	clIn,
				const char*		acKey,
				int				iNumber = -1,
				char			cDelim = '=');

// generates error messsages according to a error status 
//		returned for example by posArgForKey(..)
void util_parseError(int		iStatus,
                const char*		acKey,
                int				iNumber = -1);

// combines posArgForKey(..) and parseError(..) to one function call
void util_posArgForKeyWithCheck(
				std::istream&	clIn,
				const char*		acKey,
				int				iNumber = -1,
				char			cDelim = '=');

#endif // WITHSTREAMS
#endif // UTIL_IOFUNCTIONS_H
