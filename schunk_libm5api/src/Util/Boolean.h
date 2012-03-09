
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


#ifndef UTIL_BOOLEAN_H
#define UTIL_BOOLEAN_H

#include "Config.h"

# ifdef __GNUC__
#   if (__GNUC__ == 2 && __GNUC_MINOR__ < 8)
		#include <bool.h>
#   endif
# endif

#include <vector>
#include <iostream>

/*
	Reads boolean value from an input stream.\\
     Is called by work-around of operator
		  std::istream& operator>>(std::istream& rclStream, bool& rbBool)
	 which is missing in some C++-compilers such as
	 WATCOM and MICROSOFT (see the operator>> itself).
	rclStream: reference to input stream from which a boolean value will be read
	rbBool: reference to boolean value
	reference to the input stream which was passed as argument
*/

inline std::istream& util_readBool(std::istream& rclStream, bool& rbBool)
{
#if defined(NO_ISTREAM_OPERATOR_BOOL)
    
	#if defined(NO_CAST_FUNCTION_TEMPLATES)

		int tempL;
		rclStream >> tempL;
		rbBool = bool(tempL);
		return rclStream;
    
	#else

		int tempL;
		rclStream >> tempL;
		rbBool = static_cast<bool>(tempL);
		return rclStream;
	#endif	// NO_CAST_FUNCTION_TEMPLATES

#else

	rclStream >> rbBool;
	return rclStream;

#endif	// NO_ISTREAM_OPERATOR_BOOL
};

/*
	Reads boolean value from an input stream.
	 This inline function represents a workaround
	 for the following operator
		  std::istream& operator>>(std::istream& rclStream, bool& rbBool)
	 which is missing in some C++-compilers such as
	 WATCOM and MICROSOFT.}
	rclStream: reference to input stream from which a boolean value will be read
	rbBool: reference to boolean value
	reference to the input stream which was passed as argument
*/

#if defined(NO_ISTREAM_OPERATOR_BOOL)
#if !defined(__HAS_ISTREAM_OPERATOR_BOOL__) && (defined(__WATCOM_CPLUSPLUS__) || defined(_MSC_VER))

// check of definition __HAS_ISTREAM_OPERATOR_BOOL__ necessary due to
// name clashes with similar LEDA definition (see LEDA/param_types.h)

inline std::istream& operator>>(std::istream& rclStream, bool& rbBool)
{
        return util_readBool(rclStream, rbBool);
};

#define __HAS_ISTREAM_OPERATOR_BOOL__
#endif
#endif // NO_ISTREAM_OPERATOR_BOOL

#endif // UTIL_BOOLEAN_H
