// ========================================================================= //
//                                                                           //
//     Copyright 1999-2006											                             //
//                                                                           //
//     AMTEC robotics GmbH                                                   //
//     13127 Berlin                                                          //
//     Germany                                                               //
//                                                                           //
//     Tel   : +49 30 47 48 68 30                                            //
//     Fax   : +49 30 47 48 68 39                                            //
//     Email : support@amtec-robotics.com                                    //
//                                                                           //
// ========================================================================= //

#include "schunk_libm5api/m5apiw32.h"
#include "../Device/Device.h"
#include "../Util/IOFunctions.h"
#include "../Util/Message.h"

//#pragma data_seg(".shared")
//static int g_iM5DllRefCount = 0;
//
//
#ifdef DEBUG_M5_API
M5DLL_API int g_iM5DebugLevel = 5; //debug level
M5DLL_API int g_iM5DebugFile = 1; //write debug file
M5DLL_API int g_iM5Debug = 0; //write debug output to stdout
#else
M5DLL_API int g_iM5DebugLevel = 0;
M5DLL_API int g_iM5DebugFile = 0;
M5DLL_API int g_iM5Debug = 0;
#endif

//#pragma data_seg()

//#pragma comment(linker, "/SECTION:.shared,RWS")

static std::vector<CDevice*> g_apclDevice;
static std::vector<int> g_aiDeviceRefCount;
//HANDLE g_hM5Dll;

#if defined (_WIN32)
BOOL APIENTRY DllMain( HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved )
{	
//    BOOL bInit;
//	CMessage clDebug("M5apiw32", g_iM5DebugLevel, g_iM5Debug, g_iM5DebugFile);
//	clDebug.debug(0,"Entering M5Dll main");
	switch (ul_reason_for_call) 
    { 
        // The DLL is loading due to process 
        // initialization or a call to LoadLibrary. 
 
          case DLL_PROCESS_ATTACH: 
//			clDebug.debug(0,"Process attach M5Dll");  
            // Initialize critical section if this is the first process.
/*
	        if(g_iM5DllRefCount == 0) 
			{

				clDebug.debug(0,"Creating mutex for M5Dll");
				g_hM5Dll = CreateMutex(NULL,FALSE,"M5MUTEX");
				if(g_hM5Dll == NULL)
				{
					clDebug.debug(0,"Creation of mutex failed");
					return FALSE;
				}
				g_pcDebugFileName = "M5Debug.txt";
			}
			else
			{
				clDebug.debug(0,"Opening mutex for M5Dll");
				g_hM5Dll = OpenMutex(SYNCHRONIZE,FALSE,"M5MUTEX");
				if(g_hM5Dll == NULL)
				{
					clDebug.debug(0,"Open of mutex failed");
					return FALSE;
				}
				g_pcDebugFileName = "M5Debug.txt";
			}
*/
//			g_iM5DllRefCount++;
//			clDebug.debug(0,"M5Dll ref count %i", g_iM5DllRefCount);
            break; 
 
        // The attached process creates a new thread. 
 
        case DLL_THREAD_ATTACH: 
//			clDebug.debug(0,"Thread attach M5Dll");
//			clDebug.debug(0,"M5Dll ref count %i", g_iM5DllRefCount);
         break; 
 
        // The thread of the attached process terminates.
 
        case DLL_THREAD_DETACH: 
// 			clDebug.debug(0,"Thread detach M5Dll");
//			clDebug.debug(0,"M5Dll ref count %i", g_iM5DllRefCount);
            break; 
 
        // The DLL is unloading from a process due to 
        // process termination or a call to FreeLibrary. 
 
        case DLL_PROCESS_DETACH: 
//  		clDebug.debug(0,"Process detach M5Dll");
//			g_iM5DllRefCount--;
//			if(g_iM5DllRefCount == 0)
//				clDebug.debug(0,"Deleting mutex for M5Dll");
//			CloseHandle(g_hM5Dll);
//			clDebug.debug(0,"M5Dll ref count %i", g_iM5DllRefCount);
            break; 
 
        default: 
			break; 
     } 
 
    return TRUE; 
    UNREFERENCED_PARAMETER(hModule); 
    UNREFERENCED_PARAMETER(lpReserved); 
}
#endif

M5DLL_API int WINAPI PCube_getDllVersion()
{	
	return M5DLLVERSION;
}

M5DLL_API int WINAPI PCube_setDllDebug( int iDebug, int iDebugLevel, int iDebugFile )
{	
	g_iM5Debug = iDebug;
	g_iM5DebugLevel = iDebugFile;
	g_iM5DebugFile = iDebugLevel;
	for(int i = 0; i < g_aiDeviceRefCount.size();i++)
		if( g_apclDevice[i] != NULL )
		{
			g_apclDevice[i]->setDebug(iDebug);
			g_apclDevice[i]->setDebugLevel(iDebugLevel);
			g_apclDevice[i]->setDebugFile(iDebugFile);
		}
	return 0;
}

M5DLL_API int WINAPI PCube_configFromFile( const char* acFileName )
{
	int iRetVal, i, j;
	char acBuffer[128];
	char acInitString[128];
	char acDevice[128];
	char acModule[128];
	char acDeviceName[128];
	int iDeviceNumber = 0;
	int iDeviceStart = 0;
	int iModuleNumber = 0;
	int iModuleStart = 0;
	int iDeviceId = -1;
	int iModuleId = -1;
	float fHomeOffset = 0.0;
	float fHomeVel = 0.0;
	int iC0 = 0;
	int iDamp = 0;
	int iA0 = 0;
	float fMinPos = 0.0;
	float fMaxPos = 0.0;
	float fMaxDeltaPos = 0.0;
	float fMaxVel = 0.0;
	float fMaxAcc = 0.0;
	float fMaxCur = 0.0;
	util_searchString("PROCESS", "Debug", "0", acBuffer, 128, acFileName);
	g_iM5Debug = atoi(acBuffer);
	util_searchString("PROCESS", "DebugLevel", "0", acBuffer, 128, acFileName);
	g_iM5DebugLevel = atoi(acBuffer);
	util_searchString("PROCESS", "DebugFile", "0", acBuffer, 128, acFileName);
	g_iM5DebugFile = atoi(acBuffer);
	CMessage clDebug("M5apiw32", g_iM5DebugLevel, g_iM5Debug, g_iM5DebugFile);

	iRetVal = util_searchString("PROCESS", "DeviceNumber", "0", acBuffer, 128, acFileName);
	if(iRetVal < 0)
	{
		clDebug.warning("could not open file %s", acFileName);
		return ERRID_DEV_OPENINIFILE;
	}
	iDeviceNumber = atoi(acBuffer);
	util_searchString("PROCESS", "DeviceStart", "0", acBuffer, 128, acFileName);
	iDeviceStart = atoi(acBuffer);
	util_searchString("PROCESS", "ModuleNumber", "0", acBuffer, 128, acFileName);
	iModuleNumber = atoi(acBuffer);
	util_searchString("PROCESS", "ModuleStart", "0", acBuffer, 128, acFileName);
	iModuleStart = atoi(acBuffer);

	for(i = 0; i < iDeviceNumber; i++)
	{
		sprintf(acDevice,"DEVICE_%02d",i + iDeviceStart);
		util_searchString(acDevice, "DeviceName", "not available", acDeviceName, 128, acFileName);
		util_searchString(acDevice, "InitString", "not available", acInitString, 128, acFileName);
		clDebug.debug(0,"Trying to open device %s with initstring %s", acDeviceName, acInitString);
		iRetVal = PCube_openDevice(&iDeviceId, acInitString);
		if(iRetVal < 0)
		{
			clDebug.debug(0,"Cound not open device %s with initstring %s", acDeviceName, acInitString);
			return iRetVal;
		}
		(g_apclDevice[iDeviceId])->setName(acDeviceName);
		clDebug.debug(0,"Successfully open device %s with initstring %s", acDeviceName, acInitString);
	}

	for(i = 0; i < iModuleNumber; i++)
	{
		sprintf(acModule,"MODULE_%02d",i + iModuleStart);
		util_searchString(acModule, "DeviceName", "CAN", acDeviceName, 128, acFileName);
		util_searchString(acModule, "ModuleId", "-1", acBuffer, 128, acFileName);
		iModuleId = atoi(acBuffer);
		clDebug.debug(0,"Trying to config module %i on device %s ", iModuleId, acDeviceName);
		iDeviceId = -1;
		for(j = 0; j < g_apclDevice.size(); j++)
		{
			if(g_apclDevice[j] != NULL)
				if(strcmp(acDeviceName, (g_apclDevice[j])->getName()) == 0)
				{
					iDeviceId = j;
					break;
				}
		}
		if(iDeviceId < 0)
		{
			clDebug.debug(0,"Cound not found device %s", acDeviceName);
			return ERRID_DEV_NODEVICENAME;
		}
		util_searchString(acModule, "HomeOffset", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fHomeOffset = atof(acBuffer);
			iRetVal = PCube_setHomeOffset(iDeviceId, iModuleId, fHomeOffset);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set homeoffset of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set homeoffset to %f of module %i on device %s", fHomeOffset, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "HomeVel", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fHomeVel = atof(acBuffer);
			iRetVal = PCube_setHomeVel(iDeviceId, iModuleId, fHomeVel);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set homevel of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set homevel to %f of module %i on device %s", fHomeVel, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "C0", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			iC0 = atoi(acBuffer);
			iRetVal = PCube_setC0(iDeviceId, iModuleId, iC0);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set C0 of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set CO to %i of module %i on device %s", iC0, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "Damp", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			iDamp= atoi(acBuffer);
			iRetVal = PCube_setDamp(iDeviceId, iModuleId, iDamp);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set damp of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set damp to %i of module %i on device %s", iDamp, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "A0", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			iA0 = atoi(acBuffer);
			iRetVal = PCube_setA0(iDeviceId, iModuleId, iA0);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set A0 of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set A0 to %i of module %i on device %s", iA0, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "MinPos", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fMinPos = atof(acBuffer);
			iRetVal = PCube_setMinPos(iDeviceId, iModuleId, fMinPos);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set minpos of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set minpos to %f of module %i on device %s", fMinPos, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "MaxPos", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fMaxPos = atof(acBuffer);
			iRetVal = PCube_setMaxPos(iDeviceId, iModuleId, fMaxPos);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set maxpos of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set maxpos to %f of module %i on device %s", fMaxPos, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "MaxDeltaPos", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fMaxDeltaPos = atof(acBuffer);
			iRetVal = PCube_setMaxDeltaPos(iDeviceId, iModuleId, fMaxDeltaPos);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set maxdeltapos of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set maxdeltapos to %f of module %i on device %s", fMaxDeltaPos, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "MaxVel", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fMaxVel = atof(acBuffer);
			iRetVal = PCube_setMaxVel(iDeviceId, iModuleId, fMaxVel);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set maxvel of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set maxvel to %f of module %i on device %s", fMaxVel, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "MaxAcc", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fMaxAcc = atof(acBuffer);
			iRetVal = PCube_setMaxAcc(iDeviceId, iModuleId, fMaxAcc);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set maxacc of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set maxacc to %f of module %i on device %s", fMaxAcc, iModuleId, acDeviceName);
		}
		util_searchString(acModule, "MaxCur", "not available", acBuffer, 128, acFileName);
		if(strcmp(acBuffer, "not available") != 0)
		{
			fMaxCur = atof(acBuffer);
			iRetVal = PCube_setMaxCur(iDeviceId, iModuleId, fMaxCur);
			if(iRetVal < 0)
			{
				clDebug.debug(0,"Cound not set maxcur of module %i on device %s", iModuleId, acDeviceName);
				return iRetVal;
			}
			clDebug.debug(0,"Set maxcur to %f of module %i on device %s", fMaxCur, iModuleId, acDeviceName);
		}
		clDebug.debug(0,"Successfully config module %i on device %s ", iModuleId, acDeviceName);
	}
	return 0;
}

M5DLL_API int WINAPI PCube_openDevice( int* piDeviceId, const char* acInitString )
{
	int iRetVal = 0;
	int i, iDeviceNumber;
	bool bReplace = false;
	CMessage clDebug("M5apiw32", g_iM5DebugLevel, g_iM5Debug, g_iM5DebugFile);
	if(acInitString == NULL || strlen(acInitString) == 0)
	{
		return ERRID_DEV_NOINITSTRING;
	}
	iDeviceNumber = g_apclDevice.size();
	clDebug.debug(0,"number of possible devices %i", iDeviceNumber);
	for (i=0;i<iDeviceNumber;i++)
	{
		clDebug.debug(0,"checking device %i", i);
		if(g_apclDevice[i] != NULL)
			if(strcmp(g_apclDevice[i]->getInitString(),acInitString)==0)
			{
				*piDeviceId = i;
				g_aiDeviceRefCount[i]++;
				clDebug.debug(0,"using device with id %i", i);
				return 0;
			}
	}
	CDevice* pclDevice = newDevice(acInitString);
	if(pclDevice == NULL)
	{
		return ERRID_DEV_NODEVICENAME;
	}
	clDebug.debug(0,"created new device");
	pclDevice->setDebug(g_iM5Debug);
	pclDevice->setDebugLevel(g_iM5DebugLevel);
	pclDevice->setDebugFile(g_iM5DebugFile);
	iRetVal = pclDevice->init(acInitString);
	if(iRetVal != 0)
	{
		clDebug.debug(0,"init error device");
		delete pclDevice;
		return iRetVal;
	}

	for (i=0;i<iDeviceNumber;i++)
		if(g_apclDevice[i] == NULL)
		{
			g_apclDevice[i] = pclDevice;
			g_aiDeviceRefCount[i] = 1;
			*piDeviceId = i;
			bReplace = true;
			clDebug.debug(0,"replacing device with id %i", i);
			break;
		}
	if(bReplace == false)
	{
		g_apclDevice.push_back(pclDevice);
		g_aiDeviceRefCount.push_back(1);
		*piDeviceId = g_apclDevice.size()-1;
		clDebug.debug(0,"adding device with id %i", *piDeviceId);
	}
	return iRetVal;
}

M5DLL_API int WINAPI PCube_closeDevice( int iDeviceId )
{	
	if(0 > iDeviceId || iDeviceId >= g_aiDeviceRefCount.size())
		return ERRID_DEV_WRONGDEVICEID;
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	g_aiDeviceRefCount[iDeviceId]--;		
	if (g_aiDeviceRefCount[iDeviceId]>0)
		return 0;

	int iRetVal = g_apclDevice[iDeviceId]->exit();
	delete g_apclDevice[iDeviceId];
	g_apclDevice[iDeviceId] = NULL;
	return iRetVal;
}

M5DLL_API int WINAPI PCube_closeDevices( void )
{	
	int i;
	for(i = 0; i < g_aiDeviceRefCount.size();i++)
		g_aiDeviceRefCount[i] = 0;		
	for(i = 0; i < g_aiDeviceRefCount.size();i++)
		if( g_apclDevice[i] != NULL )
		{
			g_apclDevice[i]->exit();
			delete g_apclDevice[i];
			g_apclDevice[i] = NULL;
		}
	return 0;
}

M5DLL_API const char* WINAPI PCube_getDeviceRevision( int iDeviceId )
{	
	static const char acString[] = "";
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return acString;
	if( g_apclDevice[iDeviceId] == NULL )
		return acString;

	return g_apclDevice[iDeviceId]->getRevision();
}

M5DLL_API const char* WINAPI PCube_getDeviceName( int iDeviceId )
{	
	static const char acString[] = "";
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return acString;
	if( g_apclDevice[iDeviceId] == NULL )
		return acString;

	return g_apclDevice[iDeviceId]->getName();
}

M5DLL_API const char* WINAPI PCube_getDeviceInitString( int iDeviceId )
{	
	static const char acString[] = "";
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return acString;
	if( g_apclDevice[iDeviceId] == NULL )
		return acString;

	return g_apclDevice[iDeviceId]->getInitString();
}

M5DLL_API int WINAPI PCube_setDeviceName( int iDeviceId, const char* acDeviceName )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	g_apclDevice[iDeviceId]->setName(acDeviceName);
	return 0;
}

M5DLL_API int WINAPI PCube_setDeviceDebug( int iDeviceId, int iDebug, int iDebugLevel, int iDebugFile )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	g_apclDevice[iDeviceId]->setDebug(iDebug);
	g_apclDevice[iDeviceId]->setDebugLevel(iDebugLevel);
	g_apclDevice[iDeviceId]->setDebugFile(iDebugFile);
	return 0;
}

M5DLL_API int WINAPI PCube_getDeviceCount( void )
{	
	int iDeviceCount = 0;
	for(int i = 0; i < g_apclDevice.size(); i++)
		if( g_apclDevice[i] != NULL )
			iDeviceCount++;

	return iDeviceCount;
}

M5DLL_API int WINAPI PCube_getDeviceIdMap( int* aiIdMap )
{	
	int iDeviceCount = 0;
	for(int i = 0; i < g_apclDevice.size(); i++)
		if( g_apclDevice[i] != NULL )
		{
			*(aiIdMap++) = i;
			iDeviceCount++;
		}

	return iDeviceCount;
}

M5DLL_API int WINAPI PCube_getModuleCount( int iDeviceId )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	return g_apclDevice[iDeviceId]->getModuleCount();
}

M5DLL_API int WINAPI PCube_getModuleIdMap( int iDeviceId, int* aiIdMap )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int i;
	static std::vector<int> aiModules;
	int iRetVal = g_apclDevice[iDeviceId]->getModuleIdMap(aiModules);
	for(i = 0; i < aiModules.size(); i++)
		*(aiIdMap++) = aiModules[i];
	for(i = aiModules.size(); i < MAX_MODULES; i++)
		*(aiIdMap++) = 0;
	return iRetVal;
}

M5DLL_API int WINAPI PCube_getModuleState( int iDeviceId, int iModuleId, unsigned long* puiState )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getModuleState( iModuleId, puiState );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getModuleType( int iDeviceId, int iModuleId, unsigned char* pucValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getModuleType( iModuleId, pucValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getModuleVersion( int iDeviceId, int iModuleId, unsigned short* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getModuleVersion( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getModuleSerialNo( int iDeviceId, int iModuleId, unsigned long* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getModuleSerialNo( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefConfig( int iDeviceId, int iModuleId, unsigned long* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefConfig( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefSetup( int iDeviceId, int iModuleId, unsigned long* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefSetup( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefBaudRate( int iDeviceId, int iModuleId, unsigned char* pucValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefBaudRate( iModuleId, pucValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefBurnCount( int iDeviceId, int iModuleId, unsigned char* pucValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefBurnCount( iModuleId, pucValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefGearRatio( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefGearRatio( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefLinearRatio( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefLinearRatio( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefCurRatio( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefCurRatio( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefBrakeTimeOut( int iDeviceId, int iModuleId, unsigned short* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefBrakeTimeOut( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefIncPerTurn( int iDeviceId, int iModuleId, unsigned long* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefIncPerTurn( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefDioData( int iDeviceId, int iModuleId, unsigned long* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefDioData( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefA0( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefA0( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefC0( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefC0( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefDamp( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefDamp( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefHomeOffset( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefHomeOffset( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefHomeVel( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefHomeVel( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefMinPos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefMinPos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefMaxPos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefMaxPos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefMaxVel( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefMaxVel( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefMaxAcc( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefMaxAcc( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefMaxCur( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefMaxCur( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefMaxDeltaPos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefMaxDeltaPos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getConfig( int iDeviceId, int iModuleId, unsigned long* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getConfig( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getHomeOffset( int iDeviceId, int iModuleId, float* pValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getHomeOffset( iModuleId, pValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getHomeOffsetInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getHomeOffsetInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getIncRatio( int iDeviceId, int iModuleId, float* pValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getIncRatio( iModuleId, pValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDioData( int iDeviceId, int iModuleId, unsigned long* puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDioData( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getA0( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getA0( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getC0( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getC0( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDamp( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDamp( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getPos( int iDeviceId, int iModuleId, float* pfPos )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getPos( iModuleId, pfPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getVel( int iDeviceId, int iModuleId, float* pfVel )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getVel( iModuleId, pfVel );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getCur( int iDeviceId, int iModuleId, float* pfCur )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getCur( iModuleId, pfCur );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMinPos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMinPos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxPos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxPos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxVel( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxVel( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxAcc( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxAcc( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxCur( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxCur( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDeltaPos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDeltaPos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxDeltaPos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxDeltaPos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getSavePos( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getSavePos( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getIPolVel( int iDeviceId, int iModuleId, float* pValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getIPolVel( iModuleId, pValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getPosCountInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getPosCountInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getPosInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getPosInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getVelInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getVelInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getCurInc( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getCurInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMinPosInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMinPosInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxPosInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxPosInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxVelInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxVelInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxAccInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxAccInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDeltaPosInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDeltaPosInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxDeltaPosInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxDeltaPosInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getStateDioPos( int iDeviceId, int iModuleId, unsigned long* puiState, unsigned char* pucDio, float* pfPos )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getStateDioPos( iModuleId, puiState, pucDio, pfPos);

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getHomeVel( int iDeviceId, int iModuleId, float* pValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getHomeVel( iModuleId, pValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getHomeVelInc( int iDeviceId, int iModuleId, long* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getHomeVelInc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getSyncTime( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getSyncTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setConfig( int iDeviceId, int iModuleId, unsigned long puiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setConfig( iModuleId, puiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setHomeOffset( int iDeviceId, int iModuleId, float fValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setHomeOffset( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setHomeOffsetInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setHomeOffsetInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDioData( int iDeviceId, int iModuleId, unsigned long uiValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDioData( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setA0( int iDeviceId, int iModuleId, short iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setA0( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setC0( int iDeviceId, int iModuleId, short iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setC0( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDamp( int iDeviceId, int iModuleId, short iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDamp( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMinPos( int iDeviceId, int iModuleId, float fValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMinPos( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxPos( int iDeviceId, int iModuleId, float fValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxPos( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxVel( int iDeviceId, int iModuleId, float fValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxVel( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxAcc( int iDeviceId, int iModuleId, float fValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxAcc( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxCur( int iDeviceId, int iModuleId, float fValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxCur( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxDeltaPos( int iDeviceId, int iModuleId, float fValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxDeltaPos( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMinPosInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMinPosInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxPosInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxPosInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxVelInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxVelInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxAccInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxAccInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxDeltaPosInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxDeltaPosInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setHomeVel( int iDeviceId, int iModuleId, float fValue )			
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setHomeVel( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setHomeVelInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setHomeVelInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setRampVel( int iDeviceId, int iModuleId, float fValue )			
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setRampVel( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setRampVelInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setRampVelInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setRampAcc( int iDeviceId, int iModuleId, float fValue )			
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setRampAcc( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setRampAccInc( int iDeviceId, int iModuleId, long iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setRampAccInc( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setSyncTime( int iDeviceId, int iModuleId, short iValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setSyncTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_updateModuleIdMap( int iDeviceId )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->updateModuleIdMap();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_homeModule( int iDeviceId, int iModuleId )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->homeModule( iModuleId );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_haltModule( int iDeviceId, int iModuleId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->haltModule( iModuleId );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_resetModule( int iDeviceId, int iModuleId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->resetModule( iModuleId );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_recalcPIDParams( int iDeviceId, int iModuleId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->recalcPIDParams( iModuleId );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_movePos( int iDeviceId, int iModuleId, float fPos )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->movePos( iModuleId, fPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveRamp( int iDeviceId, int iModuleId, float fPos, float fVel, float fAcc )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveRamp( iModuleId, fPos, fVel, fAcc );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveVel( int iDeviceId, int iModuleId, float fVel )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveVel( iModuleId, fVel );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveCur( int iDeviceId, int iModuleId, float fCur )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveCur( iModuleId, fCur );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveStep( int iDeviceId, int iModuleId, float fPos, unsigned short uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveStep( iModuleId, fPos, uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_movePosInc( int iDeviceId, int iModuleId, long iPos )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->movePosInc( iModuleId, iPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveRampInc( int iDeviceId, int iModuleId, long iPos, long iVel, long iAcc )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveRampInc( iModuleId, iPos, iVel, iAcc );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveVelInc( int iDeviceId, int iModuleId, long iVel )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveVelInc( iModuleId, iVel );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveCurInc( int iDeviceId, int iModuleId, long iCur )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveCurInc( iModuleId, iCur );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveStepInc( int iDeviceId, int iModuleId, long iPos, unsigned short uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveStepInc( iModuleId, iPos, uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_movePosExtended( int iDeviceId, int iModuleId, float fPos, unsigned long* puiState, unsigned char* pucDio, float* pfPos )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->movePosExtended( iModuleId, fPos, puiState, pucDio, pfPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveRampExtended( int iDeviceId, int iModuleId, float fPos, float fVel, float fAcc, unsigned long* puiState, unsigned char* pucDio, float* pfPos )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveRampExtended( iModuleId, fPos, fVel, fAcc, puiState, pucDio, pfPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveVelExtended( int iDeviceId, int iModuleId, float fCur, unsigned long* puiState, unsigned char* pucDio, float* pfPos )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveVelExtended( iModuleId, fCur, puiState, pucDio, pfPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveCurExtended( int iDeviceId, int iModuleId, float fCur, unsigned long* puiState, unsigned char* pucDio, float* pfPos )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveCurExtended( iModuleId, fCur, puiState, pucDio, pfPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_moveStepExtended( int iDeviceId, int iModuleId, float fPos, unsigned short uiTime, unsigned long* puiState, unsigned char* pucDio, float* pfPos )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->moveStepExtended( iModuleId, fPos, uiTime, puiState, pucDio, pfPos );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_homeAll( int iDeviceId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->homeAll();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_resetAll( int iDeviceId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->resetAll();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_haltAll( int iDeviceId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->haltAll();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_serveWatchdogAll( int iDeviceId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->serveWatchdogAll();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setBaudRateAll( int iDeviceId, unsigned char ucValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setBaudRateAll(ucValue);

	return iRetVal;
}

M5DLL_API int WINAPI PCube_startMotionAll( int iDeviceId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->startMotionAll();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_savePosAll( int iDeviceId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->savePosAll();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForHomeEnd( int iDeviceId, int iModuleId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForHomeEnd( iModuleId, uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForMotionEnd( int iDeviceId, int iModuleId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForMotionEnd( iModuleId, uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForRampEnd( int iDeviceId, int iModuleId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForRampEnd( iModuleId, uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForRampDec( int iDeviceId, int iModuleId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForRampDec( iModuleId, uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForRampSteady( int iDeviceId, int iModuleId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForRampSteady( iModuleId, uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForHomeEndAll( int iDeviceId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForHomeEndAll( uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForMotionEndAll( int iDeviceId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForMotionEndAll( uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForRampEndAll( int iDeviceId, unsigned long uiTime )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForRampEndAll( uiTime );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_waitForStartMotionAll( int iDeviceId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->waitForStartMotionAll();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_xmit8Bytes( int iDeviceId, int iModuleId, void* pBytes )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->xmit8Bytes( iModuleId, pBytes );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_xack8Bytes( int iDeviceId, int iModuleId, void* pBytes )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->xack8Bytes( iModuleId, pBytes );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_doInternal( int iDeviceId, int iModuleId, void* pBytes )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->doInternal( iModuleId, pBytes );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getStateInternal( int iDeviceId, int iModuleId, unsigned long* pStat )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getStateInternal( iModuleId, pStat );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_initDLR_FTS( int iDeviceId )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->initDLR_FTS();

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDataDLR_FTS( int iDeviceId, float* fVal0, float* fVal1, float* fVal2, float* fVal3, float* fVal4, float* fVal5, long* piState )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	static std::vector<float> afVector;
	int iRetVal = g_apclDevice[iDeviceId]->getDataDLR_FTS( afVector, piState );
	*fVal0 = afVector[0];
	*fVal1 = afVector[1];
	*fVal2 = afVector[2];
	*fVal3 = afVector[3];
	*fVal4 = afVector[4];
	*fVal5 = afVector[5];

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDataSCHUNK_FTC( int iDeviceId, int iModulId, int iChannelTypeId, float* fVal0, float* fVal1, float* fVal2, float* fVal3, float* fVal4, float* fVal5, short* piState )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	static std::vector<float> afVector;
	int iRetVal = g_apclDevice[iDeviceId]->getDataSCHUNK_FTC( iModulId, iChannelTypeId, afVector, piState );
	*fVal0 = afVector[0];
	*fVal1 = afVector[1];
	*fVal2 = afVector[2];
	*fVal3 = afVector[3];
	*fVal4 = afVector[4];
	*fVal5 = afVector[5];

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setNullSCHUNK_FTC( int iDeviceId, int iModulId, short* piState )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setNullSCHUNK_FTC( iModulId, piState );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_initEMS_IO( int iDeviceId, unsigned char ucType, unsigned long uiSerialNo )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

//	int iRetVal = g_apclDevice[iDeviceId]->initEMS_IO( ucType, uiSerialNo );
//	return iRetVal;
	return ERRID_DEV_WRONGDEVICEID;
}

M5DLL_API int WINAPI PCube_getDataEMS_DIO( int iDeviceId, int iModuleId, int iChannelId, int* pData )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	bool bData = false;
//	int iRetVal = g_apclDevice[iDeviceId]->getDataEMS_IO( iModuleId, iChannelId, bData );
//	*pData = (int)bData;
//	return iRetVal;
	return ERRID_DEV_WRONGDEVICEID;
}

M5DLL_API int WINAPI PCube_setDataEMS_DIO( int iDeviceId, int iModuleId, int iChannelId, int iData )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	bool bData = (bool)iData;
//	int iRetVal = g_apclDevice[iDeviceId]->setDataEMS_IO( iModuleId, iChannelId, bData );
//	return iRetVal;
	return ERRID_DEV_WRONGDEVICEID;
}

M5DLL_API int WINAPI PCube_getDataEMS_AIO( int iDeviceId, int iModuleId, int iChannelId, float* pfData )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

//	int iRetVal = g_apclDevice[iDeviceId]->getDataEMS_IO( iModuleId, iChannelId, *pfData );
//	return iRetVal;
	return ERRID_DEV_WRONGDEVICEID;
}

M5DLL_API int WINAPI PCube_setDataEMS_AIO( int iDeviceId, int iModuleId, int iChannelId, float fData )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

//	int iRetVal = g_apclDevice[iDeviceId]->setDataEMS_IO( iModuleId, iChannelId, fData );
//	return iRetVal;
	return ERRID_DEV_WRONGDEVICEID;
}

M5DLL_API int WINAPI PCube_getDataMP55_IO( int iDeviceId, int iModuleId, float* pfData )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDataMP55_IO( iModuleId, pfData );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setTaraMP55_IO( int iDeviceId, int iModuleId, float fData )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setTaraMP55_IO( iModuleId, fData );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefCurOffset( int iDeviceId, int iModuleId, float* pfValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefCurOffset( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setInitMP55_IO_fast( int iDeviceId, int iModuleId )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setInitMP55_IO_fast( iModuleId );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDataMP55_IO_fast( int iDeviceId, int iModuleId, float* pfData )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDataMP55_IO_fast( iModuleId, pfData );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getRawMotorCurrent( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getRawMotorCurrent( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getRawMotorSupply( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getRawMotorSupply( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getRawTemperature( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getRawTemperature( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getRawLogicSupply( int iDeviceId, int iModuleId, short* piValue )
{	
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getRawLogicSupply( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getCanOpenRawAbsEnc( int iDeviceId, int iModuleId, short* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getCanOpenRawAbsEnc( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getLoadLimit( int iDeviceId, int iModuleId, long* piValue)
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getLoadLimit( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxLoadGradient( int iDeviceId, int iModuleId, long* piValue)
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxLoadGradient( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getLoadDeltaTime( int iDeviceId, int iModuleId, unsigned short* piValue)
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getLoadDeltaTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setLoadLimit( int iDeviceId, int iModuleId, long iValue)
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setLoadLimit( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxLoadGradient( int iDeviceId, int iModuleId, long iValue)
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxLoadGradient( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setLoadDeltaTime( int iDeviceId, int iModuleId, unsigned short iValue)
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setLoadDeltaTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_saveParameters( int iDeviceId, int iModuleId )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->saveParameters( iModuleId );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefCANBaudRate( int iDeviceId, int iModuleId, unsigned char* pucValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefCANBaudRate( iModuleId, pucValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getDefRSBaudRate( int iDeviceId, int iModuleId, unsigned char* pucValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getDefRSBaudRate( iModuleId, pucValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefGearRatio( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefGearRatio( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefLinRatio( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefLinRatio( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefCurRatio( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefCurRatio( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefHomeAcc( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefHomeAcc( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setModuleSerialNo( int iDeviceId, int iModuleId, unsigned long uiValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setModuleSerialNo( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefIncPerTurn( int iDeviceId, int iModuleId, unsigned long uiValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefIncPerTurn( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefBrakeTimeOut( int iDeviceId, int iModuleId, unsigned short uiValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefBrakeTimeOut( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefAddress( int iDeviceId, int iModuleId, unsigned char uiValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefAddress( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefCANBaudRate( int iDeviceId, int iModuleId, unsigned char uiValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefCANBaudRate( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefRSBaudRate( int iDeviceId, int iModuleId, unsigned char uiValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefRSBaudRate( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setDefSetup( int iDeviceId, int iModuleId, unsigned long uiValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setDefSetup( iModuleId, uiValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMotorCurrent( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMotorCurrent( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMotorSupply( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMotorSupply( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getTemperature( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getTemperature( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getLogicSupply( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getLogicSupply( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMinLogicVoltage( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMinLogicVoltage( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxLogicVoltage( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxLogicVoltage( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMinMotorVoltage( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMinMotorVoltage( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxMotorVoltage( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxMotorVoltage( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getNominalMotorCurrent( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getNominalMotorCurrent( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaximumMotorCurrent( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaximumMotorCurrent( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getLogicUndershootTime( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getLogicUndershootTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getLogicOvershootTime( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getLogicOvershootTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMotorUndershootTime( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMotorUndershootTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMotorOvershootTime( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMotorOvershootTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getNomCurOvershootTime( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getNomCurOvershootTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getHMaxCurOvershootTime( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getHMaxCurOvershootTime( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMinLogicVoltage( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMinLogicVoltage( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxLogicVoltage( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxLogicVoltage( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMinMotorVoltage( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMinMotorVoltage( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaxMotorVoltage( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaxMotorVoltage( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setNominalMotorCurrent( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setNominalMotorCurrent( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMaximumMotorCurrent( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMaximumMotorCurrent( iModuleId, fValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setLogicUndershootTime( int iDeviceId, int iModuleId, long iValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setLogicUndershootTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setLogicOvershootTime( int iDeviceId, int iModuleId, long iValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setLogicOvershootTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMotorUndershootTime( int iDeviceId, int iModuleId, long iValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMotorUndershootTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setMotorOvershootTime( int iDeviceId, int iModuleId, long iValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setMotorOvershootTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setNomCurOvershootTime( int iDeviceId, int iModuleId, long iValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setNomCurOvershootTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setHMaxCurOvershootTime( int iDeviceId, int iModuleId, long iValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setHMaxCurOvershootTime( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getKpPWMLimit( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getKpPWMLimit( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getCurrentLimit( int iDeviceId, int iModuleId, float* pfValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getCurrentLimit( iModuleId, pfValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_getMaxPWMOutput( int iDeviceId, int iModuleId, long* piValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->getMaxPWMOutput( iModuleId, piValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setKpPWMLimit( int iDeviceId, int iModuleId, long iValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setKpPWMLimit( iModuleId, iValue );

	return iRetVal;
}

M5DLL_API int WINAPI PCube_setCurrentLimit( int iDeviceId, int iModuleId, float fValue )
{
	if(0 > iDeviceId || iDeviceId >= g_apclDevice.size())
		return ERRID_DEV_WRONGDEVICEID;
	if( g_apclDevice[iDeviceId] == NULL )
		return ERRID_DEV_NOTINITIALIZED;

	int iRetVal = g_apclDevice[iDeviceId]->setCurrentLimit( iModuleId, fValue );

	return iRetVal;
}
