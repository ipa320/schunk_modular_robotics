
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
