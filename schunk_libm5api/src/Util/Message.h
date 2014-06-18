
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

extern char* g_pcDebugFileName;

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
