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
