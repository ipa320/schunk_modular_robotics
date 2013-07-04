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

#define PCTRL_CHECK_INITIALIZED()				\
  if ( isInitialized()==false )					\
    {								\
      m_ErrorMessage.assign("Manipulator not initialized.");	\
      return false;						\
    }

#define Ready4MoveStep 4638

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

	m_pc_status = PC_CTRL_OK; 
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
  int CanBaudrate = m_params->GetBaudrate();
  std::vector<double> MaxVel = m_params->GetMaxVel();
  std::vector<double> MaxAcc = m_params->GetMaxAcc();
  std::vector<double> Offsets = m_params->GetOffsets();
  std::vector<double> LowerLimits = m_params->GetLowerLimits();
  std::vector<double> UpperLimits = m_params->GetUpperLimits();

  /// Output of current settings in the terminal
  std::cout << " D  O  F  :" << DOF << std::endl;
  m_status.resize(DOF);
  m_ModuleTypes.resize(DOF);
  m_version.resize(DOF); 
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

	/// reset all modules of the chain
	int max_tries = 3; 
	for (int i = 0; i < DOF; i++)
	{  	
		for (int reset_try = 0; reset_try < max_tries; reset_try++)
		{	
			pthread_mutex_lock(&m_mutex);
			ret =  PCube_resetModule(m_DeviceHandle, ModulIDs.at(i));
			pthread_mutex_unlock(&m_mutex);
			
			if (ret == 0)
			{	
				break; 
			}
			else if ((ret != 0) && (reset_try == (max_tries-1)))
			{
				std::ostringstream errorMsg;
      	errorMsg << "Could not reset module " << ModulIDs.at(i) << " during init. Errorcode during reset: " << ret << " Try to init once more.";
      	m_ErrorMessage = errorMsg.str();
				return false;
			}
			else
			{
				// little break
				usleep(1500000); 
			}
			
		}
	}
	std::cout << "number of moduleIDs" << ModulIDs.size() << std::endl;

  /// check number of modules connected to the bus
  pthread_mutex_lock(&m_mutex);
  int number_of_modules = PCube_getModuleCount(m_DeviceHandle);
  pthread_mutex_unlock(&m_mutex);
  std::cout << "found " << number_of_modules << " modules." << std::endl;

  /// Check if the modules are connected
  for (int i = 0; i < DOF; i++)
  {
      unsigned long serNo;
      unsigned short verNo;
      unsigned long defConfig;
      std::vector<std::string> Module_Types; 
		
      /// retrieve serial number
      pthread_mutex_lock(&m_mutex);
      ret = PCube_getModuleSerialNo(m_DeviceHandle, ModulIDs[i], &serNo);
      pthread_mutex_unlock(&m_mutex);
      if (ret != 0)
			{
				std::ostringstream errorMsg;
				errorMsg << "Could not find Module with ID " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();	
				return false;
			}
		
      /// retrieve version number 		
      pthread_mutex_lock(&m_mutex);
      ret = PCube_getModuleVersion(m_DeviceHandle, ModulIDs[i], &verNo);
      pthread_mutex_unlock(&m_mutex);
      if (ret != 0)
			{
				std::ostringstream errorMsg;
				errorMsg << "Could not find Module with ID " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();	
				return false;
			}
			else
			{
			  m_version[i] = verNo; 
			}	
			
			/// retrieve defined gear ratio
			float gear_ratio; 	
      pthread_mutex_lock(&m_mutex);
      ret = PCube_getDefGearRatio(m_DeviceHandle, ModulIDs[i], &gear_ratio);
      pthread_mutex_unlock(&m_mutex);
      if (ret != 0)
			{
				std::ostringstream errorMsg;
				errorMsg << "Could not get Module type with ID " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();	
				return false;
			}
			else
			{
				if (true)
				{
			  	std::cout << "gear ratio: " << gear_ratio << std::endl; 
					//return false; 
				} 
			}	
			
			/// retrieve axis type (linear or rotational) 
			unsigned char type; 	
      pthread_mutex_lock(&m_mutex);
      ret = PCube_getModuleType(m_DeviceHandle, ModulIDs[i], &type);
      pthread_mutex_unlock(&m_mutex);
      if (ret != 0)
			{
				std::ostringstream errorMsg;
				errorMsg << "Could not get Module type with ID " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();	
				return false;
			}
			else
			{
				if (type != TYPEID_MOD_ROTARY)
				{
			  	std::cout << "wrong module type configured. Type must be rotary axis. Use Windows configuration software to change type." << std::endl; 
					return false; 
				} 
			}	

			/// find out module_type
			// the typ -if PW or PRL- can be distinguished by the typ of encoder. 
			pthread_mutex_lock(&m_mutex);
			ret = PCube_getDefSetup(m_DeviceHandle, ModulIDs[i], &defConfig);
			pthread_mutex_unlock(&m_mutex);

			ROS_DEBUG("module type check: %li (std::dec)",defConfig); 
			
  		if (ret != 0)
			{
	  		std::ostringstream errorMsg;
				errorMsg << "Error on communication with module " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();	
				return false;
			}	
		
			// Firmware version 4634 of PRL modules replies ABSOULTE_FEEDBACK, firmware 4638 replies RESOLVER_FEEDBACK. 
			// Both means the same: Module is PRL. PW modules have encoders (ENCODER_FEEDBACK, s.M5API), but this bit is not set is DefConfig word.
			// For new firmware versions this needs to be evaluated. 
			if (((defConfig & CONFIG_ABSOLUTE_FEEDBACK)==CONFIG_ABSOLUTE_FEEDBACK) || ((defConfig & CONFIG_RESOLVER_FEEDBACK)==CONFIG_RESOLVER_FEEDBACK))
			{
				m_ModuleTypes[i] = "PRL"; 
				ROS_DEBUG("Module %i is from type: PRL", i);
			}
  		else
			{
				m_ModuleTypes[i] = "PW"; 
				ROS_DEBUG("Module %i is from type: PW", i);
			}

			/// otherwise success
			std::cout << "Found module " << std::dec << ModulIDs[i] << " Serial: " << serNo << " Version: " << std::hex << verNo << std::endl;

		}
	
  // modules should be initialized now
  m_pc_status = PC_CTRL_OK;
  m_Initialized = true; 

  /// check if modules are in normal state
  std::vector<std::string> errorMessages;
  PC_CTRL_STATUS status;

  // update status variables
  updateStates();

  // grep updated status 
  getStatus(status, errorMessages);
	
  // homing dependant on moduletype and if already homed
  bool successful = false;
  successful = doHoming();
  if (!successful)
    {
      std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
      return false;
    }

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

/*
 * \brief Move joints with calculated velocities
 *
 * Calculating positions and times by desired value of the cob_trajectory_controller
 * \param velocities Vector
 */
bool PowerCubeCtrl::MoveVel(const std::vector<double>& vel)
{
  PCTRL_CHECK_INITIALIZED();
  
  //== init var ==================================================

  /// getting paramerters
	unsigned int DOF = m_params->GetDOF();
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
	
	std::vector<double> velocities; 
	velocities.resize(DOF); 
	velocities = vel; 				// temp var for velocity because of const reference of the function
	
	std::ostringstream errorMsg;			// temp error msg for being copied to m_errorMessage
	
	std::vector<std::string> errorMessages;		// temp error msg for call of getStatus()
	PC_CTRL_STATUS status;										// PowerCube Ctrl status variable

	std::vector<float> delta_pos;			// traviling distance for next command
	std::vector<float> delta_pos_horizon;
	delta_pos.resize(DOF);
	delta_pos_horizon.resize(DOF);
	
	std::vector<float> target_pos;		// absolute target postion that is desired with this command
	std::vector<float> target_pos_horizon;	
	
	target_pos.resize(DOF);	
	target_pos_horizon.resize(DOF);
	float target_time; 	// time in milliseconds
	float target_time_horizon = 0;
  
	float delta_t;			// time from the last moveVel cmd to now

	/// getting limits
	std::vector<double> LowerLimits = m_params->GetLowerLimits();
	std::vector<double> UpperLimits = m_params->GetUpperLimits();
	std::vector<double> maxVels = m_params->GetMaxVel();

	/// getting offsets
	std::vector<double> Offsets = m_params->GetOffsets();
	
	int ret; 		// temp return value holder
	float pos; 	// temp position variable for PCube_move.. cmds 

	//== calculate destination position ============================
	// needed for limit handling and MoveStep command
	
  delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
	
  m_last_time_pub = ros::Time::now();  

  std::vector<double> pos_temp;
  pos_temp.resize(DOF);

  for (unsigned int i = 0; i < DOF; i++)
  {
    // limit step time to 50msec
    //TODO: set value 0.05 as parameter
  	if (delta_t >= 0.050)
	  {
	    target_time = 0.050; //sec
	  }
	else
	  {
	    target_time = delta_t;	//sec
	  }
	
	//add horizon to time before calculation of target position, to influence both time and position at the same time
	target_time_horizon = target_time + (float)m_horizon; //sec
	// calculate travel distance
	delta_pos_horizon[i] = target_time_horizon * velocities[i];
	delta_pos[i] = target_time * velocities[i];
	
		ROS_DEBUG("delta_pos[%i]: %f target_time: %f velocitiy[%i]: %f",i ,delta_pos[i], target_time, i, velocities[i]);
 
		// calculate target position   
		target_pos_horizon[i] = m_positions[i] + delta_pos_horizon[i] - Offsets[i];
		ROS_DEBUG("target_pos[%i]: %f m_position[%i]: %f",i ,target_pos[i], i, m_positions[i]);
	}

	//== check input parameter =====================================
  
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
				// set velocities command to max value
				velocities[i] = maxVels[i];
		
				//TODO: add ros_warn 	

				ROS_INFO("Velocity %f exceeds limit %f for axis %i. moving with max velocity %f instead", velocities[i], maxVels[i], i, maxVels[i]);
			}

      /// check position limits
 			// TODO: add second limit "safty limit" 
			// if target position is outer limits and the command velocity is in in direction away from working range, skip command
      if ((target_pos[i] < LowerLimits[i]+Offsets[i]) && (velocities[i] < 0))
			{	
				ROS_INFO("Skipping command: %f Target position exceeds lower limit (%f).", target_pos[i], LowerLimits[i]);		
				// target position is set to actual position and velocity to Null. So only movement in the non limit direction is possible.
	
				pthread_mutex_lock(&m_mutex);
				PCube_haltModule(m_DeviceHandle, ModulIDs.at(i));
				pthread_mutex_unlock(&m_mutex);
				
				return true; 
			} 
			
			// if target position is outer limits and the command velocity is in in direction away from working range, skip command
			if ((target_pos[i] > UpperLimits[i]+Offsets[i]) && (velocities[i] > 0))
			{	
				ROS_INFO("Skipping command: %f Target position exceeds upper limit (%f).", target_pos[i], UpperLimits[i]);		
				// target position is set to actual position. So only movement in the non limit direction is possible.
				
				pthread_mutex_lock(&m_mutex);
				PCube_haltModule(m_DeviceHandle, ModulIDs.at(i));
				pthread_mutex_unlock(&m_mutex);

				return true; 
			} 
		}

  //== check system status ====================================== 
  
  getStatus(status, errorMessages);
	
  if ((status != PC_CTRL_OK))
  {
    m_ErrorMessage.assign("");
    for (unsigned int i = 0; i < DOF; i++)
		{
	  	m_ErrorMessage.append(errorMessages[i]);
		}
		ROS_INFO("Error during movement. Status: %i	\n", status);
    return false;
  }

  //== send velocity cmd to modules ============================== 

	//convert the time to int in [ms]
	unsigned short time4motion = (unsigned short)((target_time_horizon)*1000.0);

	for (unsigned int i = 0; i < DOF; i++)
	{
		// check module type sending command (PRL-Modules can be driven with moveStepExtended(), PW-Modules can only be driven with less safe moveVelExtended())
		if ((m_ModuleTypes.at(i) == "PRL") && (m_version[i] >= Ready4MoveStep) && !m_params->GetUseMoveVel())
		{
			//ROS_DEBUG("Modul_id = %i, ModuleType: %s, step=%f, time=%f", m_params->GetModuleID(i), m_ModuleTypes[i].c_str(), target_pos[i], target_time);

			//ROS_INFO("Modul_id = %i, ModuleType: %s, step=%f, time4motion=%i [ms], target_time=%f, horizon: %f", m_params->GetModuleID(i), m_ModuleTypes[i].c_str(), delta_pos[i], time4motion, target_time, m_horizon);
			
			pthread_mutex_lock(&m_mutex);
			ret = PCube_moveStepExtended(m_DeviceHandle, m_params->GetModuleID(i), target_pos_horizon[i], time4motion, &m_status[i], &m_dios[i], &pos);
			pthread_mutex_unlock(&m_mutex);
		}
		else	/// Types: PRL, PW, other
		{		
			pthread_mutex_lock(&m_mutex);
			ret = PCube_moveVelExtended(m_DeviceHandle, m_params->GetModuleID(i), velocities[i], &m_status[i], &m_dios[i], &pos);
			pthread_mutex_unlock(&m_mutex);
		}

		/// error handling
		if (ret != 0)
		{
			ROS_DEBUG("Com Error: %i", ret); 		  
			pos = m_positions[i];
			//m_pc_status = PC_CTRL_ERR;
			//TODO: add error msg for diagnostics if error occours often
		}
		
		// !!! Position in pos is position before moveStep movement, to get the expected position after the movement (required as input to the next moveStep command) we add the delta position (cmd_pos) !!!
		m_positions[i] = (double)pos + delta_pos[i] + Offsets[i];
		
		pos_temp[i] = (double)pos;
		//ROS_INFO("After cmd (%X) :m_positions[%i] %f = pos: %f + delta_pos[%i]: %f",m_status[i], i, m_positions[i], pos, i, delta_pos[i]);
	}

	updateVelocities(pos_temp, delta_t);
	
	//std::cout << "vel_com: " << velocities[1] << " vel_hori: " << delta_pos_horizon[1]/	target_time_horizon << " vel_real[1]: " << m_velocities.at(1) << std::endl;

	pthread_mutex_lock(&m_mutex);
	PCube_startMotionAll(m_DeviceHandle);
	pthread_mutex_unlock(&m_mutex);
	
	return true;
}

// Calculation of velocities based on vel = 1/(6*dt) * (-pos(t-3) - 3*pos(t-2) + 3*pos(t-1) + pos(t))
void PowerCubeCtrl::updateVelocities(std::vector<double> pos_temp, double delta_t)
{
  unsigned int DOF = m_params->GetDOF();

	if (m_cached_pos.empty())
	{	
		std::vector<double> null; 
		for (unsigned int i=0;i<DOF;i++){	null.push_back(0); }  

		m_cached_pos.push_back(null);
		m_cached_pos.push_back(null);
		m_cached_pos.push_back(null);
		m_cached_pos.push_back(null);
	}

	m_cached_pos.push_back(pos_temp);
  	m_cached_pos.pop_front();

	std::vector<double> last_pos = m_cached_pos.front();

    for(unsigned int i = 0; i < DOF; i++)
	{
		m_velocities[i] = 1/(6*delta_t) * (-m_cached_pos[0][i]-(3*m_cached_pos[1][i])+(3*m_cached_pos[2][i])+m_cached_pos[3][i]);
		//m_velocities[i] = (m_cached_pos[3][i] - m_cached_pos[2][i])/delta_t;		
		//m_velocities[i] = (pos_temp.at(i)-last_pos.at(i))/delta_t; 
	}   
}



/*!
 * \brief Stops the manipulator immediately
 */
bool PowerCubeCtrl::Stop()
{	
	/// getting paramerters
  unsigned int DOF = m_params->GetDOF();
	std::vector<int> ModulIDs = m_params->GetModuleIDs();

  /// stop should be executes without checking any conditions


	for (unsigned int i = 0; i < DOF; i++)
  {  
		pthread_mutex_lock(&m_mutex);
		int ret =  PCube_haltModule(m_DeviceHandle, ModulIDs.at(i));
		pthread_mutex_unlock(&m_mutex);
		if (ret != 0)
		  {
		    std::ostringstream errorMsg;
		    errorMsg << "Could not reset all modules, m5api error code: " << ret;
		    m_ErrorMessage = errorMsg.str();
		    return false;
		  }
	}

  /// after halt the modules don't accept move commands any more, they first have to be reseted
  usleep(500000);

   /// reset all modules of the chain
	for (unsigned int i = 0; i < DOF; i++)
  {  
		pthread_mutex_lock(&m_mutex);
		int ret =  PCube_resetModule(m_DeviceHandle, ModulIDs.at(i));
		pthread_mutex_unlock(&m_mutex);
		if (ret != 0)
		  {
		    std::ostringstream errorMsg;
		    errorMsg << "Could not reset all modules, m5api error code: " << ret;
		    m_ErrorMessage = errorMsg.str();
		    return false;
		  }
	}
  return true;
}

/*!
 * \brief Recovers the manipulator after an emergency stop
 */
bool PowerCubeCtrl::Recover()
{	
  unsigned int DOF = m_params->GetDOF();
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
  std::vector<double> MaxVel = m_params->GetMaxVel();
  std::vector<double> MaxAcc = m_params->GetMaxAcc();
  std::vector<double> Offsets = m_params->GetOffsets();
  
  std::vector<std::string> errorMessages;
  PC_CTRL_STATUS status;
	
  unsigned long state = PC_CTRL_OK;
  unsigned char dio;
  float position;
  int ret = 0;
	
  // check for each module if reset is necessary
  for (unsigned int i = 0; i < DOF; i++)
  {	
    pthread_mutex_lock(&m_mutex);
    ret = PCube_getStateDioPos(m_DeviceHandle, m_params->GetModuleID(i), &state, &dio, &position);
    pthread_mutex_unlock(&m_mutex);
    if (ret != 0)
		{
		  m_pc_status = PC_CTRL_ERR;
		  std::cout << "State: Error com with Module: " << i << " Time: " << ros::Time::now() << std::endl; 	
		  return false; 		
		}
		
      // if module is in error state --> reset
    if (state & STATEID_MOD_ERROR)
		{	
		  pthread_mutex_lock(&m_mutex);
		  ret = PCube_resetModule(m_DeviceHandle, m_params->GetModuleID(i));
		  pthread_mutex_unlock(&m_mutex);
		  if (ret != 0)
	    {
	      m_pc_status = PC_CTRL_ERR;
	      std::cout << "State: Error com with Module: " << i << " Time: " << ros::Time::now() << std::endl; 	
	      return false; 		
	    }
		}
  }
	
  // time for reboot
  usleep(500000); 

  // check is everything is ok now
  updateStates(); 

  if (m_pc_status == PC_CTRL_NOT_HOMED)
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
	std::vector<int> ModulIDs = m_params->GetModuleIDs();

  PCTRL_CHECK_INITIALIZED();
  for (int i = 0; i < m_params->GetDOF(); i++)
    {
      pthread_mutex_lock(&m_mutex);
      int ret = PCube_setMaxVel(m_DeviceHandle, m_params->GetModuleID(i), maxVelocity);
      pthread_mutex_unlock(&m_mutex);
			if (ret!=0)
			{	
				std::ostringstream errorMsg;
				errorMsg << "Could not set MaxVelocity in Module ID: " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();
				return false;
			}

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
	
	std::vector<int> ModulIDs = m_params->GetModuleIDs();
	
  for (int i = 0; i < m_params->GetDOF(); i++)
    {
      pthread_mutex_lock(&m_mutex);
      //std::cout << "------------------------------> PCube_setMaxVel()" << std::endl;
      int ret = PCube_setMaxVel(m_DeviceHandle, m_params->GetModuleID(i), maxVelocities[i]);
      pthread_mutex_unlock(&m_mutex);
			if (ret!=0)
			{	
				std::ostringstream errorMsg;
				errorMsg << "Could not set MaxVelocity in Module ID: " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();
				return false;
			}

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
	
	std::vector<int> ModulIDs = m_params->GetModuleIDs();

  for (int i = 0; i < m_params->GetDOF(); i++)
    {
      pthread_mutex_lock(&m_mutex);
      //std::cout << "------------------------------> PCube_setMaxAcc()" << std::endl;
      int ret = PCube_setMaxAcc(m_DeviceHandle, m_params->GetModuleID(i), maxAcceleration);
      pthread_mutex_unlock(&m_mutex);
			if (ret!=0)
			{	
				std::ostringstream errorMsg;
				errorMsg << "Could not set MaxAcceleration in Module ID: " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();
				return false;
			}

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
	
	std::vector<int> ModulIDs = m_params->GetModuleIDs();

  for (int i = 0; i < m_params->GetDOF(); i++)
    {
      pthread_mutex_lock(&m_mutex);
      int ret = PCube_setMaxAcc(m_DeviceHandle, m_params->GetModuleID(i), maxAccelerations[i]);
      pthread_mutex_unlock(&m_mutex);
			if (ret!=0)
			{	
				std::ostringstream errorMsg;
				errorMsg << "Could not set MaxAcceleration in Module ID: " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();
				return false;
			}

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
	std::vector<int> ModulIDs = m_params->GetModuleIDs();

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
	  	int ret = PCube_setConfig(m_DeviceHandle, m_params->GetModuleID(i), confword | CONFIGID_MOD_SYNC_MOTION);
	  	pthread_mutex_unlock(&m_mutex);
			if (ret!=0)
			{	
				std::ostringstream errorMsg;
				errorMsg << "Could not set SyncMotion in Module ID: " << ModulIDs[i] << ", m5api error code: " << ret;
				m_ErrorMessage = errorMsg.str();
				return false;
			}
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
  unsigned long state;
  PC_CTRL_STATUS pc_status = PC_CTRL_ERR; 
  std::vector<std::string> ErrorMessages;
  std::vector<double> Offsets = m_params->GetOffsets();
  std::ostringstream errorMsg;

  unsigned char dio;
  float position;
  int ret = 0;

  for (unsigned int i = 0; i < DOF; i++)
    {	
      state = m_status[i]; 
      pthread_mutex_lock(&m_mutex);
      ret = PCube_getStateDioPos(m_DeviceHandle, m_params->GetModuleID(i), &state, &dio, &position);
      pthread_mutex_unlock(&m_mutex);
		
      if (ret != 0)
	{
	  //m_pc_status = PC_CTRL_ERR;
	  ROS_DEBUG("Error on com in UpdateStates");
	  return true;
	  //errorMsg << "State: Error com with Module [" <<  i << "]";
	  //ErrorMessages[i] = errorMsg.str();			
	}
      else
	{	
	  ROS_DEBUG("Module %i, State: %li, Time: %f",i, state, ros::Time::now().toSec());
	  
	  m_status[i] = state; 		
	  m_dios[i] = dio;
	  m_positions[i] = position + Offsets[i];
	}	
      
    }

	/*
  double delta_t = ros::Time::now().toSec() - m_last_time_pub.toSec();
  m_last_time_pub = ros::Time::now();  

  updateVelocities(m_positions, delta_t);
	*/

  // evaluate state for translation for diagnostics msgs
  getStatus(pc_status, ErrorMessages);

  for (unsigned int i=0; i<ErrorMessages.size(); i++)
  {	
		m_ErrorMessage.clear();
		m_ErrorMessage.assign("");
    m_ErrorMessage.append(ErrorMessages[i]);
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
	
	std::vector<PC_CTRL_STATUS> StatusArray;
	StatusArray.resize(DOF);	

  errorMessages.clear();
  errorMessages.resize(DOF);

  status = PC_CTRL_OK;

  for (unsigned int i = 0; i < DOF; i++)
  {
		std::ostringstream errorMsg;
		
		if (m_status[i] & STATEID_MOD_POWERFAULT)
		{	
			if (m_status[i] & STATEID_MOD_POW_VOLT_ERR)
			{
				errorMsg << "Error in Module " << ModuleIDs[i] << ": ";
				errorMsg << "Motor voltage below minimum value! Check Emergency Stop!";
				errorMessages[i] = errorMsg.str();
			}
			else if (m_status[i] & STATEID_MOD_POW_FET_TEMP)
			{
				errorMsg << "Error in Module " << ModuleIDs[i] << ": ";
				errorMsg << "Overheated power transitors! Power must be switched of to reset this error.";
				errorMessages[i] = errorMsg.str();
			}
			else if (m_status[i] & STATEID_MOD_POW_INTEGRALERR)
			{
				errorMsg << "Error in Module " << ModuleIDs[i] << ": ";
				errorMsg << "The drive has been overloaded and the servo loop has been disabled. Power must be switched off to reset this error.";
				errorMessages[i] = errorMsg.str();
			}
			StatusArray[i] = PC_CTRL_POW_VOLT_ERR;
		}
    else if (m_status[i] & STATEID_MOD_TOW_ERROR)
		{
			errorMsg << "Error in Module " << ModuleIDs[i] << ": ";
			errorMsg << "The servo loop was not able to follow the target position!";
			errorMessages[i] = errorMsg.str();
			StatusArray[i] = PC_CTRL_ERR;
		}
    else if (m_status[i] & STATEID_MOD_ERROR)
		{	
			// STOP the motion for each module
			pthread_mutex_lock(&m_mutex);
			PCube_haltModule(m_DeviceHandle, m_params->GetModuleID(i));
			pthread_mutex_unlock(&m_mutex);
			 
			errorMsg << "Error in  Module " << ModuleIDs[i];
			errorMsg << " : Status code: " << std::hex << m_status[i];
			errorMessages[i] = errorMsg.str();
			StatusArray[i] = PC_CTRL_ERR;
		}
    else if (m_pc_status & PC_CTRL_ERR)
		{	
			Stop(); // stop all motion
			errorMsg << "PowerCubeCtrl is in global error state";
			errorMessages[i] = errorMsg.str();
			StatusArray[i] =  PC_CTRL_ERR;
		}
    else
		{
			errorMsg << "Module with Id " << ModuleIDs[i];
			errorMsg << ": Status OK.";
			errorMessages[i] = errorMsg.str();
			StatusArray[i] = PC_CTRL_OK;
		}
	}
	
	// search for worst status
	for (unsigned int i = 0; i < DOF; i++)
  {	
		if ((int)StatusArray[i] <= (int)status)
		{
			status = StatusArray[i]; 
		}	
	}
  	
	m_pc_status = status;

  return true;
}

/*!
 * \brief Gets the firmware version of the modules
 */
std::vector<unsigned long> PowerCubeCtrl::getVersion()
{
  return m_version; 
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
 * 
 * PRL-Modules are ignored. PW-Modules are checked if they are homed. If not,homeing is executed.  
 */
bool PowerCubeCtrl::doHoming()
{
	unsigned int DOF = m_params->GetDOF();
	std::vector<int> ModuleIDs = m_params->GetModuleIDs();
	std::vector<double> LowerLimits = m_params->GetLowerLimits();
	std::vector<double> UpperLimits = m_params->GetUpperLimits();

	/// getting offsets
	std::vector<double> Offsets = m_params->GetOffsets();

  /// wait until all modules are homed
  double max_homing_time = 15.0; // seconds   
  double homing_time = 999.0;	// set to 0 if any module is homed
  double intervall = 0.1;
	
  unsigned long state = PC_CTRL_ERR;
  unsigned char dio;
  float position;
  int ret = 0;
	
  /// start homing
  for (unsigned int i = 0; i < DOF; i++)
    {	
	pthread_mutex_lock(&m_mutex);
	ret = PCube_getStateDioPos(m_DeviceHandle, m_params->GetModuleID(i), &state, &dio, &position);
	pthread_mutex_unlock(&m_mutex);
		
	// check and init m_position variable for trajectory controller
	if ((position > UpperLimits[i] + Offsets[i]) || (position < LowerLimits[i] + Offsets[i]))
	{	
		std::ostringstream errorMsg;
		errorMsg << "Module " << ModuleIDs[i] << " has position " << position << " that is outside limits (" << UpperLimits[i] + Offsets[i] << " <-> " << LowerLimits[i] + Offsets[i] << std::endl; 
		if ((m_ModuleTypes.at(i)=="PW") || (m_ModuleTypes.at(i) == "other"))
		{	std::cout << "Position error for PW-Module. Init is aborted. Try to reboot the robot." << std::endl;
			m_ErrorMessage = errorMsg.str(); 
			m_pc_status = PC_CTRL_ERR;
			return false;
		}
		else if (m_ModuleTypes.at(i)=="PRL") 
		{	
			ROS_INFO("Position of Module %i is outside limits. Module can only be moved in opposite direction.",i );
			ROS_INFO("Homing for Module: %i not necessary", m_params->GetModuleID(i));
		} 	
		else 
		{	
			ROS_INFO("Module type incorrect. (in func. PowerCubeCtrl::doHoming();)");
			return false;
		}
	}
	else
	{
		m_positions[i] = position + Offsets[i]; 
	}

	// check module type before homing (PRL-Modules need not to be homed by ROS)
	if ( (m_ModuleTypes.at(i) == "PW") || (m_ModuleTypes.at(i) == "other") )
	{	
		pthread_mutex_lock(&m_mutex);
		ret = PCube_getStateDioPos(m_DeviceHandle, m_params->GetModuleID(i), &state, &dio, &position);
		pthread_mutex_unlock(&m_mutex);
	
		if (ret != 0)
		{	
			ROS_INFO("Error on communication with Module: %i.", m_params->GetModuleID(i)); 
			m_pc_status = PC_CTRL_ERR;
			return false;
		}
			
		// only do homing when necessary 
		if (!(state & STATEID_MOD_HOME))
		{
			// homing timer
			homing_time = 0.0;

			  pthread_mutex_lock(&m_mutex);
			  ret = PCube_homeModule(m_DeviceHandle, ModuleIDs[i]);
			  pthread_mutex_unlock(&m_mutex);

			  ROS_INFO("Homing started at: %f", ros::Time::now().toSec()); 

			  if (ret != 0)
			{	 
			  ROS_INFO("Error while sending homing command to Module: %i. I try to reset the module.", i); 

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
				  return false;				
				}

			}
	    }else
	    {
	      ROS_INFO("Homing for Module: %i not necessary", m_params->GetModuleID(i));
	    } 
	}else
	{
	  ROS_INFO("Homing for PRL-Module %i not necessary", m_params->GetModuleID(i));
	} 
    }

  for (unsigned int i = 0; i < DOF; i++)
    {	
      unsigned long int help; 
      do
			{
				pthread_mutex_lock(&m_mutex);
				PCube_getModuleState(m_DeviceHandle, ModuleIDs[i], &help);
				pthread_mutex_unlock(&m_mutex);
				ROS_DEBUG("Homing active for Module: %i State: %li", ModuleIDs.at(i), help);

				/// timeout watchdog for homing
				usleep(intervall * 1000000);	// convert sec to usec
				homing_time += intervall; 
				if (homing_time >= max_homing_time) {Stop(); break;} 

			} while ((help & STATEID_MOD_HOME) == 0);
     
			m_status[i] = help;
      ROS_DEBUG("State of Module %i : %li", ModuleIDs.at(i), help);
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
		
      ROS_INFO("Homing for Modul %i done.", ModuleIDs.at(i)); 
    }

  // modules successfully homed
  m_pc_status = PC_CTRL_OK;
  return true;
}

/*!
 * \brief Setup errors for diagnostics
 */
bool m_TranslateError2Diagnosics(std::ostringstream* errorMsg)
{
  return true;
}
