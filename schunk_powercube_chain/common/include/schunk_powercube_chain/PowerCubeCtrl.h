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

#ifndef __POWER_CUBE_CTRL_H_
#define __POWER_CUBE_CTRL_H_

// standard includes
#include <iostream>
#include <sstream>
#include <string>
#include <deque>
#include <pthread.h> 

// own includes
#include <schunk_libm5api/m5apiw32.h>
#include <schunk_powercube_chain/moveCommand.h>
#include <schunk_powercube_chain/PowerCubeCtrlParams.h>

class PowerCubeCtrl
{

public:

	/// Constructor
	PowerCubeCtrl(PowerCubeCtrlParams * params);

	/// Destructor
	~PowerCubeCtrl();

	typedef enum
	{
		PC_CTRL_OK = 0, PC_CTRL_NOT_HOMED = -1, PC_CTRL_ERR = -2, PC_CTRL_POW_VOLT_ERR = -3
	} PC_CTRL_STATUS;

	/////////////////////////////////////////////
	// Functions for initialization and close: //
	/////////////////////////////////////////////

	/*!
	 * \brief Initializing
	 */
	bool Init(PowerCubeCtrlParams * params);

	/*!
	 * \brief Checking if is initialized
	 */
	bool isInitialized() const
	{
		return m_Initialized;
	}

	/*!
	 * \brief Get error message
	 */
	std::string getErrorMessage() const
	{
		return m_ErrorMessage;
	}
	
	/*!
	 * \brief Get PC_Status message
	 */
	PC_CTRL_STATUS getPC_Status() const
	{
		return m_pc_status;
	}

	/*!
	 * \brief Close
	 */
	bool Close();

	////////////////////////////
	// Functions for control: //
	////////////////////////////

	/*!
	 * \brief Send position goals to powercubes, the final angles will be reached simultaneously
	 */
	bool MoveJointSpaceSync(const std::vector<double>& angles);

	/*!
	 * \brief Moves all cubes by the given velocities
	 */
	bool MoveVel(const std::vector<double>& velocities);

	/*!
	 * \brief Stops the Manipulator immediately
	 */
	bool Stop();

	/*!
	 * \brief Recovery after emergency stop or power supply failure
	 */
	bool Recover();

	//////////////////////////////////
	// functions to set parameters: //
	//////////////////////////////////

	/*!
	 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
	 *
	 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
	 */
	bool setMaxVelocity(double velocity);
	bool setMaxVelocity(const std::vector<double>& velocities);

	/*!
	 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
	 *
	 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
	 */
	bool setMaxAcceleration(double acceleration);
	bool setMaxAcceleration(const std::vector<double>& accelerations);

	/*!
	 * \brief Sets the horizon (sec).
	 *
	 * The horizon is the maximum step size which will be commanded to the powercube chain. In case
	 * of a failure this is the time the powercube chain will continue to move until it is stopped.
	 */
	bool setHorizon(double horizon);

	/*!
	 * \brief Gets the horizon (sec).
	 *
	 * The horizon is the maximum step size which will be commanded to the powercube chain. In case
	 * of a failure this is the time the powercube chain will continue to move until it is stopped.
	 */
	double getHorizon();

	/*!
	 * \brief Configure powercubes to start all movements synchronously
	 *
	 * Tells the Modules not to start moving until PCube_startMotionAll is called.
	 */
	bool setSyncMotion();

	/*!
	 * \brief Configure powercubes to start all movements asynchronously
	 *
	 * Tells the Modules to start immediately
	 */
	bool setASyncMotion();

	/////////////////////////////////////////////////
	// Functions for getting state and monitoring: //
	/////////////////////////////////////////////////

	/*!
	 * \brief Returns the state of all modules
	 */
	bool updateStates();

	/*!
	 * \brief Gets the status of the modules
	 */
	bool getStatus(PC_CTRL_STATUS& status, std::vector<std::string>& errorMessages);
	
	/*!
	 * \brief Gets the firmware version of the modules
	 */
	std::vector<unsigned long> getVersion();

	/*!
	 * \brief Returns true if any of the Joints are still moving
	 *
	 * Should also return true if Joints are accelerating or decelerating
	 */
	bool statusMoving();

	/*!
	 * \brief Gets the current positions
	 */
	std::vector<double> getPositions();

	/*!
	 * \brief Gets the current velcities
	 */
	std::vector<double> getVelocities();

	/*!
	 * \brief Gets the current accelerations
	 */
	std::vector<double> getAccelerations();

	/*!
	 * \brief Waits until all Modules are homed.
	 *
	 * Homes only Schunk PW-Modules or PRL-Modules don't need to be homed.
	 */
	bool doHoming();

	void updateVelocities(std::vector<double> pos_temp, double delta_t);

protected:
	pthread_mutex_t m_mutex;

	int m_DeviceHandle;
	bool m_Initialized;
	bool m_CANDeviceOpened;

	PowerCubeCtrlParams* m_params;
	PC_CTRL_STATUS m_pc_status;

	std::vector<unsigned long> m_status;
	std::vector<std::string> m_ModuleTypes;
	std::vector<unsigned long> m_version;
	std::vector<unsigned char> m_dios;
	std::vector<double> m_positions;
	std::deque< std::vector<double> > m_cached_pos;
	std::vector<double> m_velocities;
	std::vector<double> m_accelerations;

	double m_horizon;

	ros::Time m_last_time_pub;

	std::string m_ErrorMessage;

};

#endif
