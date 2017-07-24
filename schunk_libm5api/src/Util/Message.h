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

#ifndef UTIL_MESSAGE
#define UTIL_MESSAGE

// ---- local includes ------------------------------------------------------ ;

#include "../Util/GlobalDefines.h"
#include "../Util/InlineFunctions.h"

// ---- global includes ----------------------------------------------------- ;

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

// ---- globals ----------------------------------------------------------- ;
extern int g_iDebugLevel;

extern bool g_bDebugFile;

extern bool g_bDebug;

extern const char* g_pcDebugFileName;

// ---- typedefs ------------------------------------------------------------ ;

// ---- class definition ---------------------------------------------------- ;

class CMessage
{

private:

	// ---- private data ---------------------------------------------------- ;

	static CRITICAL_SECTION *m_csMessage;

protected:
	
	// ---- protected data -------------------------------------------------- ;

		char m_acClassName[50];
		bool m_bDebug;
		bool m_bDebugFile;
		int m_iDebugLevel;
		static double m_fInitTime;
public:

	// ---- public data ------------------------------------------------------- ;

	// ---- constructors / destructor ----------------------------------------- ;

		// default constructor
		CMessage(void);	
		CMessage(const char* pcClassName, int iDebugLevel = 0, bool bDebug = true, bool bDebugFile = false);	
		// copy constructor
		CMessage(const CMessage& clMessage);
		// destructor
		virtual ~CMessage(void);

	// ---- operators --------------------------------------------------------- ;
	
		// assignment operator
		CMessage& operator=(const CMessage& clMessage);

	// ---- query functions --------------------------------------------------- ;

		int getDebugLevel() const;

	// ---- modify functions -------------------------------------------------- ;
	
		int initMessage(const char* pcClassName, int iDebuglevel = 0, bool bDebug = true, bool bDebugFile = false);

		void setInitTime(void);
		void setDebug(bool bFlag);
		void setDebugFile(bool bFlag);
		void setDebugLevel(int iLevel);

		static void setCriticalSection(CRITICAL_SECTION *cs);

	// ---- I/O functions ----------------------------------------------------- ;

	// ---- exec functions ---------------------------------------------------- ;
		void logging(const char *pcLoggingMessage,...);

		// output of a debug message with a debug level
		void debug(const int iDebugLevel,                      
						  const char *pcDebugMessage,...) const;

		// output of a warning message
		void warning(const char *pcWarningMessage,...) const;
                      
		// output of an error message
		void error(const int iErrorCode,                      
						const char *pcErrorMessage,...) const;
		void error(const char *pcErrorMessage,...) const;
};

#endif
