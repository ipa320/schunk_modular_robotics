/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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


#include "PowerCubeSim_OROCOS.h"
#include "simulatedArm.h"
#include <vector>

using namespace RTT;

PowerCubeSim_OROCOS::PowerCubeSim_OROCOS(std::string name) : OrocosRTTArmDriverInterface(name)
{

}

PowerCubeSim_OROCOS::~PowerCubeSim_OROCOS()
{
}

bool PowerCubeSim_OROCOS::configureHook()
{
	if ( m_powercubectrl.Init(false) )
	{
		log(Info) << "PowerCubeSim initialized successfully." << endlog();
		return true;
	}
	else
	{
		log(Info) << "Error while initializing PowerCubeSim:" << endlog();
		//log(Info) << m_powercubectrl.getErrorMessage() <<  endlog();
		return false;
	}
}

bool PowerCubeSim_OROCOS::startHook()
{
 /*   if ( m_in_Angles.connected() )
    {
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Velocities.connected() || m_in_Currents.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Angles_connected = true;
    	log(Info) << "PowerCubeSim succesfully detected connection at in_Angles" << endlog();
	    return true;
    }
    if ( m_in_Velocities.connected() )
    {
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Angles.connected() || m_in_Currents.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Velocities_connected = true;
    	log(Info) << "PowerCubeSim succesfully detected connection at in_Velocities" << endlog();
	    return true;
    }
    if ( m_in_Currents.connected() )
    {
    	log(Info) << "Error, port \"setCur_R\" is connected to PowerCubeSim_OROCOS.\"";
    	log(Info) << "Current movement is not yet supported by PowerCubeSim." << endlog();
    	return false;
    	/*
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Angles.connected() || m_in_Velocities.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Current_connected = true;
	    return true;

    }
    else
    {
    	log(Info) << "No Input port is connected, PowerCubeSim will only return the current state." << endlog();
    }    */
    return true;
}

void PowerCubeSim_OROCOS::updateHook()
{
	//log(Info) << "updateHook is being executed." << endlog();
	Jointd setpositions, setvelocities;
	setpositions = set_position_inport.Get();
	setvelocities = set_velocity_inport.Get();
	if(setpositions.size() == 7)
	{

	}
	else if(setvelocities.size() == 7)
	{

	}

	current_position_outport.Set(m_powercubectrl.getConfig());
	current_velocity_outport.Set(m_powercubectrl.getJointVelocities());
}

void PowerCubeSim_OROCOS::stopHook()
{
	stopArm();
}

bool PowerCubeSim_OROCOS::stopArm()
{
    //stop
    m_powercubectrl.stop();
    return true;
}

bool PowerCubeSim_OROCOS::isArmStopped()
{
    //isStopped
    if(m_powercubectrl.statusMoving())
        return false;
    return true;
}
