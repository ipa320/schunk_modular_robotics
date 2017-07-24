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

#ifndef DEVICEERRORS_H
#define DEVICEERRORS_H

	// Device Errors
	
#define	ERRID_DEV							(-200L)		// device message

#define ERRID_DEV_FUNCTIONNOTAVAILABLE  	(ERRID_DEV - 1)	
#define ERRID_DEV_NOINITSTRING				(ERRID_DEV - 2)
#define ERRID_DEV_NODEVICENAME				(ERRID_DEV - 3)
#define ERRID_DEV_BADINITSTRING				(ERRID_DEV - 4)
#define ERRID_DEV_INITERROR					(ERRID_DEV - 5)
#define ERRID_DEV_NOTINITIALIZED			(ERRID_DEV - 6)
#define ERRID_DEV_WRITEERROR				(ERRID_DEV - 7)
#define ERRID_DEV_READERROR					(ERRID_DEV - 8)
#define ERRID_DEV_WRITETIMEOUT				(ERRID_DEV - 9)
#define ERRID_DEV_READTIMEOUT				(ERRID_DEV - 10)
#define ERRID_DEV_WRONGMESSAGEID			(ERRID_DEV - 11)
#define ERRID_DEV_WRONGCOMMANDID			(ERRID_DEV - 12)
#define ERRID_DEV_WRONGPARAMETERID			(ERRID_DEV - 13)
#define ERRID_DEV_EXITERROR					(ERRID_DEV - 14)
#define ERRID_DEV_NOMODULES					(ERRID_DEV - 15)
#define ERRID_DEV_WRONGDEVICEID				(ERRID_DEV - 16)
#define ERRID_DEV_NOLIBRARY					(ERRID_DEV - 17)
#define ERRID_DEV_ISINITIALIZED				(ERRID_DEV - 18)
#define ERRID_DEV_WRONGEMSMODULEID			(ERRID_DEV - 19)
#define ERRID_DEV_EMSNOTINITIALIZED			(ERRID_DEV - 20)
#define ERRID_DEV_EMSMAXNUMBER				(ERRID_DEV - 21)
#define ERRID_DEV_EMSINITERROR				(ERRID_DEV - 22)
#define ERRID_DEV_WRONGEMSTYPE				(ERRID_DEV - 23)
#define ERRID_DEV_WRONGEMSCHANNELID			(ERRID_DEV - 24)
#define ERRID_DEV_WRONGMP55MODULEID			(ERRID_DEV - 25)
#define ERRID_DEV_WRONGSCHUNKMODULEID		(ERRID_DEV - 26)
#define ERRID_DEV_WRONGMODULEID				(ERRID_DEV - 27)
#define ERRID_DEV_MODULEERROR				(ERRID_DEV - 28)
#define ERRID_DEV_WAITTIMEOUT				(ERRID_DEV - 29)
#define ERRID_DEV_CONNECTIONNOTINIT			(ERRID_DEV - 30)
#define ERRID_DEV_PARSERERROR				(ERRID_DEV - 31)
#endif
