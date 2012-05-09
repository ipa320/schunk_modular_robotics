/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: schunk_modular_robotics
 * \note
 *   ROS stack name: schunk_modular_robotics
 * \note
 *   ROS package name: schunk_powercube_chain
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Dec 2010
 *
 * \brief
 *   Implementation of powercube control.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

// ROS includes
#include <ros/ros.h>

// own includes
#include <schunk_powercube_chain/PowerCubeCtrl.h>

#define PCTRL_CHECK_INITIALIZED() \
if ( isInitialized()==false )											\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}

/*
 * \brief Constructor
 */
PowerCubeCtrl::PowerCubeCtrl(PowerCubeCtrlParams * params)
{
	m_mutex = PTHREAD_MUTEX_INITIALIZER;

	m_CANDeviceOpened = false;
	m_Initialized = false;

	m_params = params;

	m_horizon = 0.01; // sec

	m_last_time_pub = ros::Time::now();
}

/*
 * \brief Destructor
 */
PowerCubeCtrl::~PowerCubeCtrl()
{		
	// stop all components
	Stop(); 
	
	// close CAN device
	if (m_CANDeviceOpened)
	{
	  pthread_mutex_lock(&m_mutex);
	  PCube_closeDevice(m_DeviceHandle);
	  pthread_mutex_unlock(&m_mutex);
	}
}

/// ToDo: Check brief
/*!
 * \brief Initializing
 *
 * Setting paramters initialized by PowerCubeCtrlParams.h
 */
bool PowerCubeCtrl::Init(PowerCubeCtrlParams * params)
{
	int ret = 0;
	int DOF = m_params->GetDOF();
	std::string CanModule = m_params->GetCanModule();
	std::string CanDevice = m_params->GetCanDevice();
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
	std::vector<std::string> ModuleTypes = m_params->GetModuleTypes();
	int CanBaudrate = m_params->GetBaudrate();
	std::vector<double> MaxVel = m_params->GetMaxVel();
	std::vector<double> MaxAcc = m_params->GetMaxAcc();
	std::vector<double> Offsets = m_params->GetOffsets();
	std::vector<double> LowerLimits = m_params->GetLowerLimits();
	std::vector<double> UpperLimits = m_params->GetUpperLimits();

	/// Output of current settings in the terminal
	std::cout << " D  O  F  :" << DOF << std::endl;
	m_status.resize(DOF);
	m_dios.resize(DOF);
	m_positions.resize(DOF);
	m_velocities.resize(DOF);

	std::cout << "=========================================================================== " << std::endl;
	std::cout << "PowerCubeCtrl:Init: Trying to initialize with the following parameters: " << std::endl;
	std::cout << "DOF: " << DOF << std::endl;
	std::cout << "CanModule: " << CanModule << std::endl;
	std::cout << "CanDevice: " << CanDevice << std::endl;
	std::cout << "CanBaudrate: " << CanBaudrate << std::endl;
	std::cout << "ModulIDs: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << ModulIDs[i] << " ";
	}
	std::cout << std::endl;

	std::cout << "ModuleTypes: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << ModuleTypes.at(i) << " ";
	}
	std::cout << std::endl << "maxVel: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << MaxVel[i] << " ";
	}

	std::cout << std::endl << "maxAcc: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << MaxAcc[i] << " ";
	}

	std::cout << std::endl << "upperLimits: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << UpperLimits[i] << " ";
	}

	std::cout << std::endl << "lowerLimits: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << LowerLimits[i] << " ";
	}

	std::cout << std::endl << "offsets: ";
	for (int i = 0; i < DOF; i++)
	{
		std::cout << Offsets[i] << " ";
	}

	std::cout << std::endl << "=========================================================================== " << std::endl;
	std::ostringstream InitStr;
	InitStr << CanModule << ":" << CanDevice << "," << CanBaudrate;
	std::cout << "initstring = " << InitStr.str().c_str() << std::endl;

	/// open device
	pthread_mutex_lock(&m_mutex);
	ret = PCube_openDevice(&m_DeviceHandle, InitStr.str().c_str());
	pthread_mutex_unlock(&m_mutex);
	if (ret != 0)
	{
		std::ostringstream errorMsg;
		errorMsg << "Could not open device " << CanDevice << ", m5api error code: " << ret;
		m_ErrorMessage = errorMsg.str();
		return false;
	}

	m_CANDeviceOpened = true;

	/// reset all modules
	pthread_mutex_lock(&m_mutex);
	ret = PCube_resetAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);
	if (ret != 0)
	{
		std::ostringstream errorMsg;
		errorMsg << "Could not reset all modules, m5api error code: " << ret;
		m_ErrorMessage = errorMsg.str();
		return false;
	}

	/// make sure m_IdModules is clear of Elements:
	ModulIDs.clear();

	/// check number of modules connected to the bus
	pthread_mutex_lock(&m_mutex);
	int number_of_modules = PCube_getModuleCount(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);
	std::cout << "found " << number_of_modules << " modules." << std::endl;

	/// Check if the modules are connected
	for (int i = 0; i < DOF; i++)
	{
		unsigned long serNo;

		pthread_mutex_lock(&m_mutex);
		ret = PCube_getModuleSerialNo(m_DeviceHandle, ModulIDs[i], &serNo);
		pthread_mutex_unlock(&m_mutex);
		if (ret != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not find Module with ID " << ModulIDs[i] << ", m5api error code: " << ret;
			m_ErrorMessage = errorMsg.str();	
			//return false;
		}
		/// otherwise success
		std::cout << "Found module " << ModulIDs[i] << " Serial: " << serNo << std::endl;
	}

	// modules should be initialized now
	m_pc_status = PC_CTRL_OK;
	m_Initialized = true; 

	// check if modules are in normal state
	std::vector<std::string> errorMessages;
	PC_CTRL_STATUS status;

 	// update status variables
	updateStates();

	// grep updated status 
	getStatus(status, errorMessages);
	
	if ((status != PC_CTRL_OK) && (status != PC_CTRL_NOT_HOMED))
	{
		m_ErrorMessage.assign("");
		for (int i = 0; i < DOF; i++)
		{
			m_ErrorMessage.append(errorMessages[i]);			
		}
		return false;
	}
	else if (status == PC_CTRL_NOT_HOMED)
	{
		std::cout << "PowerCubeCtrl:Init: Homing is executed ...\n";
		bool successful = false;
		successful = doHoming();
		if (!successful)
		{
			std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
			return false;
		}
	}

	/// Set angle offsets to hardware
	for (int i = 0; i < DOF; i++)
	{
		pthread_mutex_lock(&m_mutex);
		//std::cout << "------------------------------> PCube_setHomeOffset()" << std::endl;
		PCube_setHomeOffset(m_DeviceHandle, ModulIDs[i], Offsets[i]);
		pthread_mutex_unlock(&m_mutex);
	}

	/// Set limits to hardware
	for (int i = 0; i < DOF; i++)
	{
		pthread_mutex_lock(&m_mutex);
		PCube_setMinPos(m_DeviceHandle, ModulIDs[i], LowerLimits[i]);
		pthread_mutex_unlock(&m_mutex);

		pthread_mutex_lock(&m_mutex);
		PCube_setMaxPos(m_DeviceHandle, ModulIDs[i], UpperLimits[i]);
		pthread_mutex_unlock(&m_mutex);
	}

	/// Set max velocity to hardware
	setMaxVelocity(MaxVel);

	/// Set max acceleration to hardware
	setMaxAcceleration(MaxAcc);

	/// set synchronous or asynchronous movements
	setSyncMotion();
	//setASyncMotion();

	// All modules initialized successfully
	m_pc_status = PC_CTRL_OK;

	return true;
}

/*!
 * \brief Close CAN devices
 */
bool PowerCubeCtrl::Close()
{
	if (m_CANDeviceOpened)
	{
		m_Initialized = false;
		m_CANDeviceOpened = false;

		pthread_mutex_lock(&m_mutex);
		PCube_closeDevice(m_DeviceHandle);
		pthread_mutex_unlock(&m_mutex);

		return true;
	}

	else
	{
		return false;
	}
}

/// ToDo: Check comments
/*!
 * \brief Move joints synchronous
 *
 * Adjusting velocity of all joints to reach the final angules at the same time
 */
bool PowerCubeCtrl::MoveJointSpaceSync(const std::vector<double>& target)
{
	PCTRL_CHECK_INITIALIZED();
	unsigned int DOF = m_params->GetDOF();

	std::vector<std::string> errorMessages;
	PC_CTRL_STATUS status;
	getStatus(status, errorMessages);
	if ((status != PC_CTRL_OK))
	{
		m_ErrorMessage.assign("");
		for (unsigned int i = 0; i < DOF; i++)
		{
			m_ErrorMessage.append(errorMessages[i]);
		}
		return false;
	}

	std::vector<double> vel(DOF);
	std::vector<double> acc(DOF);

	double TG = 0;

	try
	{
		/// calculate which joint takes the longest time to reach goal
		std::vector<double> times(DOF);
		for (unsigned int i = 0; i < DOF; i++)
		{
			RampCommand rm(m_positions[i], m_velocities[i], target[i], m_params->GetMaxAcc()[i],
						   m_params->GetMaxVel()[i]);
			times[i] = rm.getTotalTime();
		}

		/// determine the joint index that has the greatest value for time
		int furthest = 0;
		double max = times[0];
		for (unsigned int i = 1; i < DOF; i++)
		{
			if (times[i] > max)
			{
				max = times[i];
				furthest = i;
			}
		}

		RampCommand rm_furthest(m_positions[furthest], m_velocities[furthest], target[furthest],
                                m_params->GetMaxAcc()[furthest], m_params->GetMaxVel()[furthest]);

		double T1 = rm_furthest.T1();
		double T2 = rm_furthest.T2();
		double T3 = rm_furthest.T3();

		/// total time:
		TG = T1 + T2 + T3;

		/// calculate velocity and acceleration for all joints:
		acc[furthest] = m_params->GetMaxAcc()[furthest];
		vel[furthest] = m_params->GetMaxVel()[furthest];
		for (unsigned int i = 0; i < DOF; i++)
		{
			if (int(i) != furthest)
			{
				double a;
				double v;
				RampCommand::calculateAV(m_positions[i], m_velocities[i], target[i], TG, T3, m_params->GetMaxAcc()[i],
										 m_params->GetMaxVel()[i], a, v);

				acc[i] = a;
				vel[i] = v;
			}
		}
	}
	catch (...)
	{
		return false;
	}

	/// Send motion commands to hardware
	for (unsigned int i = 0; i < DOF; i++)
	{
		pthread_mutex_lock(&m_mutex);
		PCube_moveRamp(m_DeviceHandle, m_params->GetModuleIDs()[i], target[i], fabs(vel[i]), fabs(acc[i]));
		pthread_mutex_unlock(&m_mutex);
	}

	pthread_mutex_lock(&m_mutex);
	PCube_startMotionAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	return true;
}

/*
 * \brief Move joints with calculated velocities
 *
 * Calculating positions and times by desired value of the cob_trajectory_controller
 * \param velocities Vector
 */

/// ToDo: moveStep still really unstable

bool PowerCubeCtrl::MoveVel(const std::vector<double>& velocities)
{
	PCTRL_CHECK_INITIALIZED();

	unsigned int DOF = m_params->GetDOF();

	/// getting limits
	std::vector<double> lowerLimits = m_params->GetLowerLimits();
	std::vector<double> upperLimits = m_params->GetUpperLimits();
	std::vector<double> maxVels = m_params->GetMaxVel();
	/// check dimensions
	if (velocities.size() != DOF)
	{
		m_ErrorMessage = "Skipping command: Commanded velocities and DOF are not same dimension.";
		return false;
	}

	for (unsigned int i = 0; i < DOF; i++)
	{
	  /// check velocity limit
	  if(velocities[i] > maxVels[i])
	    {
	      //velocities[i] = maxVels[i];
		std::ostringstream errorMsg;
		errorMsg << "Velocity " << velocities[i] << " exceeds limit " << maxVels[i] << "for axis " << i << " moving with " << maxVels[i] << " instead";
		m_ErrorMessage = errorMsg.str();
	
	    }
	  /// check position limits
	  double targetpos = m_positions[i] + (0.02 * velocities[i]);
	  if (targetpos < lowerLimits[i] || targetpos > upperLimits[i])
		{
			std::ostringstream errorMsg;
			errorMsg << "Skipping command: Target position " << targetpos << " exceeds limit " << lowerLimits[i] << " - " << upperLimits[i] << "for axis " << i;
			m_ErrorMessage = errorMsg.str();
			return false;
		}
	}

	std::vector<std::string> errorMessages;
	PC_CTRL_STATUS status;
	getStatus(status, errorMessages);
	if ((status != PC_CTRL_OK))
	{
		m_ErrorMessage.assign("");
		for (unsigned int i = 0; i < DOF; i++)
		{
			m_ErrorMessage.append(errorMessages[i]);
		}
		return false;
	}

	float delta_t;
  	delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
	m_last_time_pub = ros::Time::now();  

	std::vector<double> pos_temp;
	pos_temp.resize(DOF);

	for (unsigned int i = 0; i < DOF; i++)
	{
		float pos;
		float cmd_pos;
		double  cmd_time; // time in milliseconds
 
	    	// limit step time to 20msec
	    	if (delta_t >= 0.050)
		{
		  cmd_time = 0.050; //sec
	    	}
		else
		  cmd_time = delta_t; //sec
		cmd_pos = cmd_time * velocities[i];
		pthread_mutex_lock(&m_mutex);
		//std::cout << "Modul_id = " << m_params->GetModuleID(i) <<", step= "<< m_positions[i] + cmd_pos << ", time = " << cmd_time << std::endl;
		//int ret = PCube_moveVelExtended(m_DeviceHandle, m_params->GetModuleID(i), velocities[i], &m_status[i], &m_dios[i], &pos);
		int ret = PCube_moveStepExtended(m_DeviceHandle, m_params->GetModuleID(i), m_positions[i] + cmd_pos, (cmd_time+m_horizon), &m_status[i], &m_dios[i], &pos);
		pthread_mutex_unlock(&m_mutex);

		if (ret != 0)
		{
			ROS_INFO("Com Error"); 		  
			pos = m_positions[i];
		  //std::cout << "MoveVelExt retuned: " << ret << std::endl;
		  //	m_pc_status = PC_CTRL_ERR;
		}else
		{ROS_INFO("worked");}
		
		// !!! Position in pos is position before moveStep movement, to get the expected position after the movement (required as input to the next moveStep command) we add the delta position (cmd_pos) !!!
		m_positions[i] = (double)pos + cmd_pos;
		pos_temp[i] = (double)pos;
		
	}
	// Calculation of velocities based on vel = 1/(6*dt) * (-pos(t-3) - 3*pos(t-2) + 3*pos(t-1) + pos(t))
	if(m_cached_pos.size() < 4)
	  {
	    m_cached_pos.push_back(pos_temp);
	    for(unsigned int i = 0; i < DOF; i++)
	      m_velocities[i] = 0.0;
	  }
	else
	  {
	    m_cached_pos.push_back(pos_temp);
	    m_cached_pos.pop_front();
	    for(unsigned int i = 0; i < DOF; i++)
	      {
		m_velocities[i] = 1/(6*delta_t) * (-m_cached_pos[0][i]-(3*m_cached_pos[1][i])+(3*m_cached_pos[2][i])+m_cached_pos[3][i]);
	      }
	  }

	pthread_mutex_lock(&m_mutex);
	PCube_startMotionAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	return true;
}

/*!
 * \brief Stops the manipulator immediately
 */
bool PowerCubeCtrl::Stop()
{
	/// stop should be executes without checking any conditions
	pthread_mutex_lock(&m_mutex);
	PCube_haltAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	/// after halt the modules don't accept move commands any more, they first have to be reseted
	usleep(500000);
	pthread_mutex_lock(&m_mutex);
	PCube_resetAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	return true;
}

/*!
 * \brief Recovers the manipulator after an emergency stop
 */
bool PowerCubeCtrl::Recover()
{
	std::vector<std::string> errorMessages;
	PC_CTRL_STATUS status;

	pthread_mutex_lock(&m_mutex);
	PCube_haltAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	usleep(500000);

	pthread_mutex_lock(&m_mutex);
	PCube_resetAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);
	
	usleep(500000);

	updateStates(); 

	getStatus(status, errorMessages);
	if (status == PC_CTRL_NOT_HOMED)
	{
		if (!doHoming())
		{
			return false;
		}
	}
	
	usleep(500000);

	// modules should be recovered now
	m_pc_status = PC_CTRL_OK;	
	
	updateStates(); 
	// check if modules are really back to normal state
	getStatus(status, errorMessages);

	if ((status != PC_CTRL_OK))
	{
		m_ErrorMessage.assign("");
		for (int i = 0; i < m_params->GetDOF(); i++)
		{
			m_ErrorMessage.append(errorMessages[i]);
		}
		return false;
	}

	/// modules successfully recovered
	m_pc_status = PC_CTRL_OK;
	return true;
}

/*!
 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
 *
 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
 */
bool PowerCubeCtrl::setMaxVelocity(double maxVelocity)
{
	PCTRL_CHECK_INITIALIZED();
	for (int i = 0; i < m_params->GetDOF(); i++)
	{
		pthread_mutex_lock(&m_mutex);
		PCube_setMaxVel(m_DeviceHandle, m_params->GetModuleID(i), maxVelocity);
		pthread_mutex_unlock(&m_mutex);

		std::vector<double> maxVelocities(maxVelocity);
		m_params->SetMaxVel(maxVelocities);
	}

	return true;
}

/*!
 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
 */
bool PowerCubeCtrl::setMaxVelocity(const std::vector<double>& maxVelocities)
{
	PCTRL_CHECK_INITIALIZED();

	for (int i = 0; i < m_params->GetDOF(); i++)
	{
		pthread_mutex_lock(&m_mutex);
		//std::cout << "------------------------------> PCube_setMaxVel()" << std::endl;
		PCube_setMaxVel(m_DeviceHandle, m_params->GetModuleID(i), maxVelocities[i]);
		pthread_mutex_unlock(&m_mutex);
		m_params->SetMaxVel(maxVelocities);
	}

	return true;
}

/*!
 * \brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
 *
 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
 */
bool PowerCubeCtrl::setMaxAcceleration(double maxAcceleration)
{
	PCTRL_CHECK_INITIALIZED();

	for (int i = 0; i < m_params->GetDOF(); i++)
	{
		pthread_mutex_lock(&m_mutex);
		//std::cout << "------------------------------> PCube_setMaxAcc()" << std::endl;
		PCube_setMaxAcc(m_DeviceHandle, m_params->GetModuleID(i), maxAcceleration);
		pthread_mutex_unlock(&m_mutex);
		std::vector<double> maxAccelerations(maxAcceleration);
		m_params->SetMaxAcc(maxAccelerations);
	}

	return true;
}

/*!
 * \brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
 */
bool PowerCubeCtrl::setMaxAcceleration(const std::vector<double>& maxAccelerations)
{
	PCTRL_CHECK_INITIALIZED();

	for (int i = 0; i < m_params->GetDOF(); i++)
	{
		pthread_mutex_lock(&m_mutex);
		PCube_setMaxAcc(m_DeviceHandle, m_params->GetModuleID(i), maxAccelerations[i]);
		pthread_mutex_unlock(&m_mutex);
		m_params->SetMaxAcc(maxAccelerations);
	}

	return true;
}

/*!
 * \brief Sets the horizon (sec).
 *
 * The horizon is the maximum step size which will be commanded to the powercube chain. In case
 * of a failure this is the time the powercube chain will continue to move until it is stopped.
 */
bool PowerCubeCtrl::setHorizon(double horizon)
{
	m_horizon = horizon;

	return true;
}

/*!
 * \brief Gets the horizon (sec).
 *
 * The horizon is the maximum step size which will be commanded to the powercube chain. In case
 * of a failure this is the time the powercube chain will continue to move until it is stopped.
 */
double PowerCubeCtrl::getHorizon()
{
	return m_horizon;
}

/*!
 * \brief Configure powercubes to start all movements synchronously
 *
 * Tells the Modules not to start moving until PCube_startMotionAll is called.
 */
bool PowerCubeCtrl::setSyncMotion()
{
	if (m_CANDeviceOpened)
	{
		for (int i = 0; i < m_params->GetDOF(); i++)
		{
			unsigned long confword;

			/// get config
			pthread_mutex_lock(&m_mutex);
			PCube_getConfig(m_DeviceHandle, m_params->GetModuleID(i), &confword);
			pthread_mutex_unlock(&m_mutex);

			/// set config to synchronous
			pthread_mutex_lock(&m_mutex);
			PCube_setConfig(m_DeviceHandle, m_params->GetModuleID(i), confword | CONFIGID_MOD_SYNC_MOTION);
			pthread_mutex_unlock(&m_mutex);
		}
		return true;
	}
	else
	{
		return false;
	}
}
/*!
 * \brief Configure powercubes to start all movements asynchronously
 *
 * Tells the Modules to start immediately
 */
bool PowerCubeCtrl::setASyncMotion()
{
	if (m_CANDeviceOpened)
	{
		for (int i = 0; i < m_params->GetDOF(); i++)
		{
			unsigned long confword;

			/// get config
			pthread_mutex_lock(&m_mutex);
			PCube_getConfig(m_DeviceHandle, m_params->GetModuleID(i), &confword);
			pthread_mutex_unlock(&m_mutex);

			/// set config to asynchronous
			pthread_mutex_lock(&m_mutex);
			PCube_setConfig(m_DeviceHandle, m_params->GetModuleID(i), confword & (~CONFIGID_MOD_SYNC_MOTION));
			pthread_mutex_unlock(&m_mutex);
		}
		return true;
	}

	else
	{
		return false;
	}
}
/*!
 * \brief Returns the current states
 */
bool PowerCubeCtrl::updateStates()
{
	PCTRL_CHECK_INITIALIZED();

	unsigned int DOF = m_params->GetDOF();

	unsigned long state = PC_CTRL_ERR;
	
	unsigned long state_s = PC_CTRL_ERR; // for debug

	unsigned char dio;
	float position;
	int ret = 0;
	for (unsigned int i = 0; i < DOF; i++)
	{	
		state_s = m_status[i]; 
		pthread_mutex_lock(&m_mutex);
		ret = PCube_getStateDioPos(m_DeviceHandle, m_params->GetModuleID(i), &state, &dio, &position);
		pthread_mutex_unlock(&m_mutex);
		
		if (ret != 0)
		{
			//m_pc_status = PC_CTRL_ERR;
			//std::cout << "State: Error com with Module: " << i << " Time: " << ros::Time::now() << std::endl; 			
		}
		else
		{	
			if( state_s != state)
			{
				std::cout << "State: " << state << " Joint: " << i << std::endl; 
			}
		 	m_status[i] = state;
			m_dios[i] = dio;
			m_positions[i] = position;
		}	
		
    /// TODO: calculate vel and acc
    ///m_velocities = ???;
    ///m_accelerations = ???
	}
	return true;
}

/*!
 * \brief Gets the status of the modules
 */
bool PowerCubeCtrl::getStatus(PC_CTRL_STATUS& status, std::vector<std::string>& errorMessages)
{
	unsigned int DOF = m_params->GetDOF();
	std::vector<int> ModuleIDs = m_params->GetModuleIDs();
	std::vector<std::string> ModuleTypes = m_params->GetModuleTypes();
	
	errorMessages.clear();
	errorMessages.resize(DOF);

	status = PC_CTRL_ERR;

	for (unsigned int i = 0; i < DOF; i++)
	{
		std::ostringstream errorMsg;

		if (m_status[i] & STATEID_MOD_POW_VOLT_ERR)
		{
			errorMsg << "Error in Module " << ModuleIDs[i] << ": ";
			errorMsg << "Motor voltage below minimum value!";
			errorMessages[i] = errorMsg.str();
			status = PC_CTRL_POW_VOLT_ERR;
		}
		// check if module is homed (necessary for Schunk PW-Modules, but not possible with PRL-Modules)
		// TODO: Can this be used to avoid referece problems with PRL-Modules? (TIF)
		else if ((!(ModuleTypes.at(i) == "PRL")) && ((m_status[i] & STATEID_MOD_HOME) == 0))
		{
		  errorMsg << "Warning: Module " << ModuleIDs[i];
		  errorMsg << " is not referenced!";
		  errorMessages[i] = errorMsg.str();
		  status = PC_CTRL_NOT_HOMED;
		}

		else if (m_status[i] & STATEID_MOD_ERROR)
		{
			errorMsg << "Error in  Module " << ModuleIDs[i];
			errorMsg << " : Status code: " << std::hex << m_status[i];
			errorMessages[i] = errorMsg.str();
			status = PC_CTRL_ERR;
		}
		else if (m_pc_status & PC_CTRL_ERR)
		{
			errorMsg << "PowerCubeCtrl is in global error state";
			errorMessages[i] = errorMsg.str();
			status =  PC_CTRL_ERR;
		}
		else
		{
			errorMsg << "Module with Id " << ModuleIDs[i];
			errorMsg << ": Status OK.";
			errorMessages[i] = errorMsg.str();
			status = PC_CTRL_OK;
		}
	}

	m_pc_status = status;

	return true;
}

/*!
 * \brief Returns true if some cubes are still moving
 */
bool PowerCubeCtrl::statusMoving()
{
	PCTRL_CHECK_INITIALIZED();

	for (int i = 0; i < m_params->GetDOF(); i++)
	{
		if (m_status[i] & STATEID_MOD_MOTION)
			return true;
	}
	return false;
}

/*!
 * \brief Gets the current positions
 */
std::vector<double> PowerCubeCtrl::getPositions()
{
	return m_positions;
}

/*!
 * \brief Gets the current velocities
 */
std::vector<double> PowerCubeCtrl::getVelocities()
{
	/// ToDo: calculate new velocities before returning
	return m_velocities;
}

/*!
 * \brief Gets the current positions
 */
std::vector<double> PowerCubeCtrl::getAccelerations()
{
	/// ToDo: calculate new accelerations before returning
	return m_accelerations;
}

/*!
 * \brief Does homing for all Modules
 */
bool PowerCubeCtrl::doHoming()
{
	unsigned int DOF = m_params->GetDOF();
	std::vector<int> ModuleIDs = m_params->GetModuleIDs();
	std::vector<std::string> ModuleTypes = m_params->GetModuleTypes();
	
	/// wait until all modules are homed
	double max_homing_time = 15.0; // seconds   
	double homing_time = 999.0;	// set to 0 if any module is homed
	double intervall = 0.1;
	
	/// start homing
	int ret = 0;
	for (unsigned int i = 0; i < DOF; i++)
	{	
		// check if homing for module is permitted (PRL-Modules need not to be homed by ROS)
		if ( (ModuleTypes.at(i) == "PW") || (ModuleTypes.at(i) == "other") )
		{	// homing timer
			homing_time = 0.0;

			pthread_mutex_lock(&m_mutex);
			ret = PCube_homeModule(m_DeviceHandle, ModuleIDs[i]);
			pthread_mutex_unlock(&m_mutex);

			std::cout << " homing started at " << ros::Time::now() << std::endl; 

			if (ret != 0)
			{	// TODO: sure that ret must be 0 ???  
				std::cout << "Error while sending homing command to Module: " << i << "I try to reset the module." << std::endl; 

				// reset module with the hope that homing works afterwards
				pthread_mutex_lock(&m_mutex);
				ret = PCube_resetModule(m_DeviceHandle, ModuleIDs[i]);
				pthread_mutex_unlock(&m_mutex);
				if (ret != 0)
				{	
					std::ostringstream errorMsg;
					errorMsg << "Can't reset module after homing error" << ModuleIDs[i] << ", m5api error code: " << ret;
					m_ErrorMessage = errorMsg.str();
				}

				// little break for reboot
				usleep(200000); 

				pthread_mutex_lock(&m_mutex);
				ret = PCube_homeModule(m_DeviceHandle, ModuleIDs[i]);
				pthread_mutex_unlock(&m_mutex);
				if (ret != 0)
				{	
					std::ostringstream errorMsg;
					errorMsg << "Can't start homing for module " << ModuleIDs[i] << ", tried reset with no success, m5api error code: " << ret;
					m_ErrorMessage = errorMsg.str();
					std::cout << "Error on sending homing command to module " << ModuleIDs.at(i) << "!" << std::endl;		
					return false;				
				}
			}	
		}
	}

	for (unsigned int i = 0; i < DOF; i++)
	{	
		
		// check if homing for module is permitted (PRL-Modules need not to be homed by ROS)
		//if ( (ModuleTypes.at(i) == "PW") || (ModuleTypes.at(i) == "other") )
		//{
			unsigned long int help;
			do
			{
				pthread_mutex_lock(&m_mutex);
				PCube_getModuleState(m_DeviceHandle, ModuleIDs[i], &help);
				pthread_mutex_unlock(&m_mutex);
				std::cout << "homing active Module:" << ModuleIDs.at(i) << ": " << help << std::endl;
				/// convert sec to usec
				usleep(intervall * 1000000);
				homing_time += intervall; 
				if (homing_time >= max_homing_time) {Stop(); break;} 
			} while ((help & STATEID_MOD_HOME) == 0);
			m_status[i] = help;
			std::cout << "State of Module " << ModuleIDs.at(i) << ": " << help << std::endl;
		//} 
	}

	for (unsigned int i = 0; i < DOF; i++)
	{	
			/// check result
			if (!(m_status[i] & STATEID_MOD_HOME) || (m_status[i] & STATEID_MOD_ERROR) )
			{
				std::cout << "Homing failed: Error in  Module " << ModuleIDs[i] << std::endl;
				m_pc_status = PC_CTRL_NOT_HOMED;
				return false;
			}
		
		std::cout << "Homing for Modul " << ModuleIDs.at(i) << " done." << std::endl; 
	}

	// modules successfully homed
	m_pc_status = PC_CTRL_OK;
	return true;
}
