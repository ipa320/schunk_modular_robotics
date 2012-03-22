
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


#include "IOFunctions.h"
#include <string.h>

int util_ignore(int iSize, char cDelimiter, FILE* hFileHandle)
{
	char cChar;
	for(int i = 0; i < iSize; i++)
	{
		cChar = fgetc(hFileHandle);
		if(cChar == EOF)
			return -1;
		if(cChar == cDelimiter)
			return 0;
	}
	return 0;
}

int util_skipWhiteSpace(FILE* hFileHandle)
{
	char cChar;
	do
	{
		cChar = fgetc(hFileHandle);
		if(cChar == EOF)
			return -1;
		if(cChar != ' ' && cChar != '"' && cChar != '\t')
		{
			ungetc(cChar, hFileHandle);
			return 0;
		}
	}while(1);
	return 0;
}

int util_getStringCutWhiteSpace(char* acReturnString, int iSize, FILE* hFileHandle)
{
	char cChar;
	fgets(acReturnString, iSize, hFileHandle);
	for(int i = 0; i < iSize; i++)
	{
		cChar = acReturnString[i];
		if(cChar == ' ' || cChar == '"' || cChar == '#' || cChar == ';' || cChar == '\t' || cChar == '\r' || cChar == '\n' || cChar == '\0')
		{
			acReturnString[i] = '\0';
			break;
		}
	}
	return 0;
}

int util_searchSection(const char* acSectionName, FILE* hFileHandle)
{
	int iRetVal = 1;
	int iSectionLength;
	char cChar;
	char acBuffer[512];
	iSectionLength = strlen(acSectionName);
	do
	{
		cChar = fgetc(hFileHandle);
		if(cChar == EOF)						// check for end of file
			iRetVal = -1;
		else if(cChar == '#' || cChar == ';')	// check for comment
		{
			iRetVal = util_ignore(0x7FFF,'\n', hFileHandle);		// skip all characters up to '\n'
			if(iRetVal == 0)
				iRetVal = 1;
		}
		else if(cChar == '[')					// no comment so parse it
		{
			fgets(acBuffer, iSectionLength+1, hFileHandle);

			if(strncmp(acBuffer, acSectionName, iSectionLength) == 0)
			{
				cChar = fgetc(hFileHandle);
				if(cChar == ']')
					iRetVal = util_ignore(0x7FFF,'\n', hFileHandle);	// skip all characters up to '\n'
				else
					iRetVal = -1;
			}
		}
	}
	while(iRetVal > 0);
	return iRetVal;
}

int util_searchKey(const char* acKeyName, FILE* hFileHandle)
{
	int iRetVal = 1;
	int iKeyLength, iBufferLength;
	char cChar;
	char acBuffer[512];
	iKeyLength = strlen(acKeyName);
	do
	{
		cChar = fgetc(hFileHandle);
		if(cChar == EOF)						// check for end of file
			iRetVal = -1;
		else if(cChar == '[')					// check for new section
		{
			ungetc(cChar, hFileHandle);
			iRetVal = -1;
		}
		else if(cChar == '#' || cChar == ';')	// check for comment
		{
			iRetVal = util_ignore(0x7FFF,'\n', hFileHandle);		// skip all characters up to '\n'
			if(iRetVal == 0)
				iRetVal = 1;
		}
		else if(cChar != ' ' && cChar != '\t' && cChar != '\r' && cChar != '\n')	// no comment or whitespace so parse it
		{
			acBuffer[0] = cChar;
			if (iKeyLength > 1)
				fgets(acBuffer + 1, iKeyLength, hFileHandle);

			iBufferLength = strlen(acBuffer);
			if(iBufferLength > iKeyLength || (iBufferLength == iKeyLength && acBuffer[iBufferLength - 1] != '\n'))
			{
				if(strncmp(acBuffer, acKeyName, iKeyLength) == 0)
				{
					iRetVal = util_skipWhiteSpace(hFileHandle);
					cChar = fgetc(hFileHandle);
					if(cChar == '=')			 // check for delimiter
						iRetVal = util_skipWhiteSpace(hFileHandle);
					else
						return -1;
				}
				else
				{
					iRetVal = util_ignore(0x7FFF,'\n', hFileHandle);		// skip all characters up to '\n'
					if(iRetVal == 0)
						iRetVal = 1;
				}
			}
		}
	}
	while(iRetVal > 0);
	return iRetVal;
}

int util_searchString(const char* acSectionName, const char* acKeyName, const char* acDefaultString, char* acReturnString, int iSize, const char* acFileName)
{
	FILE* hFileHandle = fopen( acFileName, "r" );
	if(hFileHandle <= 0)
	{
		strncpy(acReturnString, acDefaultString, iSize);
		return -1;
	}

	if(util_searchSection(acSectionName, hFileHandle) < 0)
	{
		strncpy(acReturnString, acDefaultString, iSize);
		fclose(hFileHandle);
		return 0;
	}
	if(util_searchKey(acKeyName, hFileHandle) < 0)
	{
		strncpy(acReturnString, acDefaultString, iSize);
		fclose(hFileHandle);
		return 0;
	}
	util_getStringCutWhiteSpace(acReturnString, iSize, hFileHandle);
	fclose(hFileHandle);
	return strlen(acReturnString);
}

int util_setSection(const char* acSectionName, FILE* hFileHandle)
{
	int iRetVal = fseek(hFileHandle,0,SEEK_CUR);
	if(iRetVal < 0)
	{
//		std::cout << "Section set error" << std::endl;
		return -1;
	}
	iRetVal = fprintf(hFileHandle, "\n\n[%s]", acSectionName);
	if(iRetVal == strlen(acSectionName) + 4)
	{
//		std::cout << "Section set" << std::endl;
		fseek(hFileHandle,0,SEEK_CUR);
		return 0;
	}
	else
	{
//		std::cout << "Section set error" << std::endl;
		return -1;
	}
}

int util_setKey(const char* acKeyName, FILE* hFileHandle)
{
	int iRetVal = fseek(hFileHandle,0,SEEK_CUR);
	if(iRetVal < 0)
	{
//		std::cout << "Section set error" << std::endl;
		return -1;
	}
	iRetVal = fprintf(hFileHandle, "\n%s = ", acKeyName);
	if(iRetVal == strlen(acKeyName) + 4)
	{
//		std::cout << "Key set" << std::endl;
		fseek(hFileHandle,0,SEEK_CUR);
		return 0;
	}
	else
	{
//		std::cout << "Key set error" << std::endl;
		return -1;
	}
}

int util_setString(const char* acSectionName, const char* acKeyName, const char* acString, const char* acFileName)
{
	int iRetVal = 0;
	int iLength = 0;
	char* acBuffer = NULL;
	FILE* hFileHandle = fopen( acFileName, "r+" );
	if(hFileHandle <= 0)
	{
		hFileHandle = fopen( acFileName, "w+" );
		if(hFileHandle <= 0)
		{
//			std::cout << "File open error" << std::endl;
			return -1;
		}
	}
//	std::cout << "File open" << std::endl;
	if(util_searchSection(acSectionName, hFileHandle) < 0)
	{
//		std::cout << "Section not found" << std::endl;
		iRetVal = util_setSection(acSectionName, hFileHandle);
		if(iRetVal < 0)
		{
			fclose(hFileHandle);
			return -1;
		}
	}
//	else
//		std::cout << "Section found" << std::endl;
	if(util_searchKey(acKeyName, hFileHandle) < 0)
	{
//		std::cout << "Key not found" << std::endl;
		fpos_t fposRest;
		iRetVal = fgetpos(hFileHandle, &fposRest);
		if(iRetVal < 0)
		{
//			std::cout << "get Rest pos error" << std::endl;
			fclose(hFileHandle);
			return -1;
		}
		char cChar;
		do
		{
#if defined(__LINUX__)
			fposRest.__pos--;
#else
			fposRest--;
#endif
			iRetVal = fsetpos(hFileHandle, &fposRest);
			if(iRetVal < 0)
			{
//				std::cout << "set Rest pos error" << std::endl;
				fclose(hFileHandle);
				return -1;
			}
			cChar = fgetc(hFileHandle);
			if(cChar != '\n')
			{
#if defined(__LINUX__)
				fposRest.__pos++;
#else
				fposRest++;
#endif
				iRetVal = fsetpos(hFileHandle, &fposRest);
				if(iRetVal < 0)
				{
//					std::cout << "set Rest pos error" << std::endl;
					fclose(hFileHandle);
					return -1;
				}
				break;
			}
		}while(1);

		do
		{
			cChar = fgetc(hFileHandle);
			if(cChar == EOF)
				break;
		}while(1);
		fpos_t fposEnd;
		iRetVal = fgetpos(hFileHandle, &fposEnd);
		if(iRetVal < 0)
		{
//			std::cout << "get End pos error" << std::endl;
			fclose(hFileHandle);
			return -1;
		}
#if defined(__LINUX__)
		iLength = fposEnd.__pos - fposRest.__pos;
#else
		iLength = fposEnd - fposRest;
#endif
		if(iLength > 0)
		{
			acBuffer = new char[iLength];
	//		std::cout << "Rest length: " << iLength << std::endl;

			iRetVal = fsetpos(hFileHandle, &fposRest);
			if(iRetVal < 0)
			{
	//			std::cout << "set Rest pos error" << std::endl;
				fclose(hFileHandle);
				if(acBuffer != NULL)
					delete[] acBuffer;
				return -1;
			}
					
			iLength = fread(acBuffer, sizeof(char), iLength, hFileHandle);
			if(iLength < 0)
			{
	//			std::cout << "read Rest error" << std::endl;
				fclose(hFileHandle);
				if(acBuffer != NULL)
					delete[] acBuffer;
				return -1;
			}
			acBuffer[iLength] = '\0';
		}
//		std::cout << "read Rest:" << acBuffer << std::endl;
		iRetVal = fsetpos(hFileHandle, &fposRest);
		if(iRetVal < 0)
		{
//			std::cout << "set String pos error" << std::endl;
			fclose(hFileHandle);
			if(acBuffer != NULL)
				delete[] acBuffer;
			return -1;
		}
		iRetVal = util_setKey(acKeyName, hFileHandle);
		if(iRetVal < 0)
		{
			fclose(hFileHandle);
			if(acBuffer != NULL)
				delete[] acBuffer;
			return -1;
		}
		iRetVal = fprintf(hFileHandle, "%s", acString);
		if(iRetVal != strlen(acString))
		{
//			std::cout << "String set error" << std::endl;
			fclose(hFileHandle);
			if(acBuffer != NULL)
				delete[] acBuffer;
			return -1;
		}
//		else
//			std::cout << "String set" << std::endl;
		if(iLength > 0)
		{
			iLength = fwrite(acBuffer, sizeof(char), iLength, hFileHandle);
			if(iLength != iLength)
			{
	//			std::cout << "write Rest error" << std::endl;
				fclose(hFileHandle);
				if(acBuffer != NULL)
					delete[] acBuffer;
				return -1;
			}
			if(acBuffer != NULL)
				delete[] acBuffer;
		}
	}
	else
	{	
//		std::cout << "Key found" << std::endl;
		fpos_t fposString;
		iRetVal = fgetpos(hFileHandle, &fposString);
		if(iRetVal < 0)
		{
//			std::cout << "get String pos error" << std::endl;
			fclose(hFileHandle);
			return -1;
		}
		
		char cChar;
		int iErase = 0;
		do
		{
			cChar = fgetc(hFileHandle);
			if(cChar == EOF || cChar == '\n' || cChar == ';' || cChar == '#')
			{
				ungetc(cChar, hFileHandle);
				break;
			}
			iErase++;
		}while(1);
		fpos_t fposRest;
		iRetVal = fgetpos(hFileHandle, &fposRest);
		if(iRetVal < 0)
		{
//			std::cout << "get Rest pos error" << std::endl;
			fclose(hFileHandle);
			return -1;
		}

		do
		{
			cChar = fgetc(hFileHandle);
			if(cChar == EOF)
				break;
		}while(1);
		fpos_t fposEnd;
		iRetVal = fgetpos(hFileHandle, &fposEnd);
		if(iRetVal < 0)
		{
			std::cout << "get End pos error" << std::endl;
			fclose(hFileHandle);
			return -1;
		}
#if defined(__LINUX__)
		iLength = fposEnd.__pos - fposRest.__pos;
#else
		iLength = fposEnd - fposRest;
#endif
		if(iLength > 0)
		{
			acBuffer = new char[iLength];
	//		std::cout << "Rest length: " << iLength << std::endl;

			iRetVal = fsetpos(hFileHandle, &fposRest);
			if(iRetVal < 0)
			{
	//			std::cout << "set Rest pos error" << std::endl;
				fclose(hFileHandle);
				if(acBuffer != NULL)
					delete[] acBuffer;
				return -1;
			}
					
			iLength = fread(acBuffer, sizeof(char), iLength, hFileHandle);
			if(iLength < 0)
			{
	//			std::cout << "read Rest error" << std::endl;
				fclose(hFileHandle);
				if(acBuffer != NULL)
					delete[] acBuffer;
				return -1;
			}
			acBuffer[iLength] = '\0';
		}
//		std::cout << "read Rest:" << acBuffer << std::endl;
		iRetVal = fsetpos(hFileHandle, &fposString);
		if(iRetVal < 0)
		{
//			std::cout << "set String pos error" << std::endl;
			fclose(hFileHandle);
			if(acBuffer != NULL)
				delete[] acBuffer;
			return -1;
		}

		iRetVal = fprintf(hFileHandle, "%s ", acString);
		iErase -= strlen(acString) + 1;
		if(iRetVal != strlen(acString) + 1)
		{
//			std::cout << "String set error" << std::endl;
			fclose(hFileHandle);
			if(acBuffer != NULL)
				delete[] acBuffer;
			return -1;
		}
//		else
//			std::cout << "String set" << std::endl;

		if(iLength > 0)
		{
			iLength = fwrite(acBuffer, sizeof(char), iLength, hFileHandle);
			if(iLength != iLength)
			{
	//			std::cout << "write Rest error" << std::endl;
				fclose(hFileHandle);
				if(acBuffer != NULL)
					delete[] acBuffer;
				return -1;
			}
	//		std::cout << "erase " << iErase << std::endl;
			for(int i = 0; i < iErase; i++)
			{
				cChar = fputc('\0', hFileHandle);
				if(cChar == EOF)
					break;
			}
			if(acBuffer != NULL)
				delete[] acBuffer;
		}
	}
	fclose(hFileHandle);
	return 0;
}

// -------------------------------------------------------------------------- ;

//\fdd{ This function helps to read input formatted as keyword (or phrase),
//		number (optional), delimiter, followed by an argument, which will be
//		read by some other function. Leading white space and comments
//		(everything from a '#' or ';' sign until the end of that line) will be
//		skipped. The next std::string (including any white space) will be compared
//		against the keyword. If this does not match the function returns
//		imediately and indicates the error. If the number is positive
//		(including zero) the next std::string after skipping white space will be
//		read in as an integral number. If the numbers are not equal the
//		function returns and indicates the error. Otherwise white space will
//		be skipped and the next character will be compared with the delimiter.
//		In case of mismatch the function returns an error code, otherwise it
//		skips white space a last time. Now the stream should be positioned at
//		the argument, which could be read by an adequate read function. }
//\xmp{	The following examples could be processed by this function:
//		\begin{itemize}
//			\item	\verb|count of events = 5|
//			\item	\verb|item 001: first item|
//			\item	\verb|position = [ 12, 34 ]|
//			\item	\verb|circle:|\newline
//					\verb|center = [ 43, 21]|\newline
//					\verb|radius = 7|
//		\end{itemize}
//		In the last example \texttt{circle} is the keyword, no number is used
//		and the delimiter is a colon. Therefore the corresponding call to this
//		function would look as follows:\newline
//		\begin{quote}
//			\verb|error = posArgForKey(inL, "circle", -1, ':');|\newline
//		\end{quote}
//		This function will check for "circle" and the colon. Then it positions
//		the stream just ahead of the std::string "center". Given a class
//		\texttt{circle} with a method read able to process this format, we
//		could just call:\newline
//		\begin{quote}
//			\verb|circle::read(inL);|\newline
//		\end{quote}
//		in order to read the circle from the input stream. }
//\arg{	in:		a reference to an input stream }
//\arg{	key:	the keyword (or phrase) to look for }
//\arg{	number: an optional number following the keyword }
//\arg{	delim:	the sign preceeding the argument for the given keyword }
//\ret{	The constant \texttt{OKAY} indicates successful operation. In case of
//		an error the following cases are distinguished:
//		\begin{itemize}
//			\item	\texttt{NO\_KEY} the keyword did not match
//			\item	\texttt{KEY\_BUT\_WRONG\_NUMBER} the number did not match
//			\item	\texttt{KEY\_BUT\_NO\_EQUAL} the delimiter did not match
//			\item	\texttt{FOUND\_EOF} EOF reached
//		\end{itemize}. }

#ifdef WITHSTREAMS
int util_posArgForKey(
		std::istream&		in,
		const char*		key,
		int				number,
		char			delim)
{
	static char buf[BUFFER_LENGTH];
           char cL;
           int  count;
	
	while( !in.eof() )
	{
		in >> cL;

		if(cL == '#' || cL == ';')	// check for comment
			in.ignore(0x7FFF,'\n'); // skip all characters up to '\n'
		else						// no comment so parse it
		{
			buf[0] = cL;

			if (strlen(key) > 1)  // Workaround for bug in WATCOM's std::istream::get
				in.get(buf+1, strlen(key), '\n');

			if(strncmp(buf, key, strlen(key)) == 0)
			{
				if (number >= 0)
				{
					in >> count;
					
					if (count != number)
						return KEY_BUT_WRONG_NUMBER;
				};
				
				in >> std::ws;         // skip whitespace
				in >> cL;
				if(cL == delim)	  // check for delimiter
				{
					// skip whitespace
					
					in >> std::ws;
					
					// the argument should follow now
					
					return OKAY;
				}
				else
					return KEY_BUT_NO_EQUAL;
			}	
			else
				return NO_KEY;
		};
	};

	return FOUND_EOF;
};	

// -------------------------------------------------------------------------- ;
/* \fdd{Generates error messsages according to an error status for example generated 
by posArgForKey(..)} 
*/
void util_parseError(
		int				status,
		const char*		key,
		int				number)
{
	switch(status)
	{
		case OKAY:
			break;
		case KEY_BUT_NO_EQUAL:
			std::cerr << "\nread(in) parse error : '=' expected behind";
			std::cerr << key;
			if (number >= 0)
				std::cerr << " " << number;
			std::cerr << " !";
			break;
		case NO_KEY:
			std::cerr << "\nread(in) parse error : '";
			std::cerr << key;
			if (number >= 0)
				std::cerr << " " << number;
			std::cerr << "' expected !";
			break;
		case FOUND_EOF:
			std::cerr << "\nread(in) parse error : premature EOF '";
			std::cerr << key;
			if (number >= 0)
				std::cerr << " " << number;
			std::cerr << "' expected !";
			break;
		case NO_OPEN_BRACKET:
			std::cerr << "\nread(in) parse error : '[' expected before";
			std::cerr << key;
			if (number >= 0)
				std::cerr << " " << number;
			std::cerr << " argument !";
			break;
		case NO_SEPERATOR:
			std::cerr << "\nread(in) parse error : ', ' expected ";
			std::cerr << " between components of " << key;
			if (number >= 0)
				std::cerr << " " << number;
			std::cerr << " argument !";
			break;
		case NO_CLOSED_BRACKET:
			std::cerr << "\nread(in) parse error : ']' expected behind";
			std::cerr << key;
			if (number >= 0)
				std::cerr << " " << number;
			std::cerr << " argument !";
			break;
		default:
			std::cerr << "\nread(in) : unknown error !?!?!?!?!?!?!?!?!";
			break;
	};
};

// -------------------------------------------------------------------------- ;
/*
\fdd{combines posArgForKey(..) and parseError(..) to one function call.}
\arg{std::istream& inA}
\arg{const char* const keyA}
\arg{int numberA}
\arg{char delimA}
\ret{{\tt const int} is returned. If eof occured before anything else than 
  FOUND\_EOF is returned. If no key was found NO\_KEY is returned. 
  If a key was found but the wrong number followed KEY\_BUT\_WRONG\_NUMBER is returned.
  If a key was found, the number was negativ and no equal sign followed KEY\_BUT\_NO\_EQUAL 
  is returned. }
*/
void util_posArgForKeyWithCheck(
		std::istream&	in, 
		const char*		key, 
		int				number, 
		char			delim )
{
	int        status;
	
	status = util_posArgForKey(in, key, number, delim);
	
	if(status != OKAY)
		util_parseError(status, key);
};

#endif
