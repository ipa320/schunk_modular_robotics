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

#ifndef UTIL_CONFIG_H
#define UTIL_CONFIG_H

// This config file is intended to contain all necessary configuration
// switches to get the software running on all compilers/platforms. It should
// be included by EVERY file as the first include file !!!!!!!!!
//
// ========================================================================= //



// -------------------------------------------------------------------------- ;
// Linux
// -------------------------------------------------------------------------- ;
#ifdef __LINUX__
	#ifndef	LINUX
	#define LINUX
	#endif
#endif

#ifdef __LINUX__

#define __386__
// already defined as inline in <cmath>
#define WITHSTREAMS
#define HAS_ABS_FCT

#endif // Linux 

// -------------------------------------------------------------------------- ;
// Sun Workshop compiler : 
// -------------------------------------------------------------------------- ;

#if defined (__SUNPRO_CC)

#define WITHSTREAMS
#define NO_CLASS_PARTIAL_SPECIALIZATION
#define NO_MEMBER_TEMPLATES 
#define NO_DEFAULT_ARGS_FOR_FUNCTION_TEMPLATES
#define NO_ABS_FCT

#endif // __SUNPRO_CC

// -------------------------------------------------------------------------- ;
// Watcom compiler : 
// -------------------------------------------------------------------------- ;

#if defined (__WATCOM_CPLUSPLUS__)

#define WITHSTREAMS
#define __386__
#define NO_CLASS_PARTIAL_SPECIALIZATION
#define NO_MEMBER_TEMPLATES 
#define NO_DEFAULT_ARGS_FOR_FUNCTION_TEMPLATES
#define NO_CAST_FUNCTION_TEMPLATES
//#define NO_ISTREAM_OPERATOR_BOOL
#define NO_STL_STRINGS
#define NO_ABS_FCT
#define std

#endif // __WATCOM_CPLUSPLUS__

// -------------------------------------------------------------------------- ;

// -------------------------------------------------------------------------- ;
// Microsoft compiler : 
// -------------------------------------------------------------------------- ;
#if defined(_MSC_VER)

#define WITHSTREAMS				// comment if using MFC !!!
#define __386__
#if (_MSC_VER <= 1200)
#define NO_CLASS_PARTIAL_SPECIALIZATION
#define NO_MEMBER_TEMPLATES 
#endif

#if (_MSC_VER < 1200)
#define NO_ISTREAM_OPERATOR_BOOL
#endif

#endif // _MSC_VER

// -------------------------------------------------------------------------- ;

#endif // UTIL_CONFIG_H
