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
