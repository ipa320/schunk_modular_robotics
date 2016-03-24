// BASED ON NOT WORKING EXAMPLE OF NEOBOTIX!

#ifndef SocketCANDevice_INCLUDEDEF_H
#define SocketCANDevice_INCLUDEDEF_H
//-----------------------------------------------

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <cerrno>
#include <cstring>

#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <string.h>

#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif


//-----------------------------------------------

#include <iostream>
#include <cstdio>
#include <libpcan/libpcan.h>
#include "../Device/ProtocolDevice.h"

//-----------------------------------------------

class SocketCANDevice: public CProtocolDevice {

private:

	// ---- private data ---------------------------------------------------- ;

	// ---- private auxiliary functions ------------------------------------- ;

protected:
	// ---- protected data ---------------------------------------------------- ;
	bool m_bInitialized;
	int m_iDeviceId;
	int m_iNoOfRetries;
	unsigned short m_uiQueueSize;
	unsigned long m_uiTimeOut;
	char * m_DeviceName;


	// ---- protected auxiliary functions ------------------------------------- ;
	int getDeviceError(int iErrorState);
	int setBaudRate();
	int setBaudRate(unsigned char iBaudRate);
	int setMessageId(unsigned long uiMessageId);
	int clearReadQueue();
	int reinit(unsigned char ucBaudRateId);
	int readDevice(CProtocolMessage& rclProtocolMessage);
	int writeDevice(CProtocolMessage& rclProtocolMessage);

public:

	// ---- public data ----------------------------------------------------- ;

	// ---- constructors / destructor --------------------------------------- ;

	// default constructor
	SocketCANDevice();
	SocketCANDevice(const SocketCANDevice& rclSocketCANDevice);
	~SocketCANDevice();
	
	// ---- operators ------------------------------------------------------- ;
	// assignment operator
	SocketCANDevice& operator=(const SocketCANDevice& rclSocketCANDevice);
	
	// ---- query functions ------------------------------------------------- ;
	
	// ---- modify functions ------------------------------------------------ ;

	void setQueueSize(unsigned short uiQueueSize);
	void setTimeOut(unsigned long uiTimeOut);

	// ---- I/O functions --------------------------------------------------- ;

	// ---- exec functions -------------------------------------------------- ;

	int init();
	int init(unsigned long baudRate);
	int init(const char* acInitString);
	int exit();
	int waitForStartMotionAll();


};
#endif
