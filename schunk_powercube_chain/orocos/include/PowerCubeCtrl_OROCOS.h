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


#ifndef _POWERCUBECTRL_OROCOS_
#define _POWERCUBECTRL_OROCOS_

#include <rtt/TaskContext.hpp>
#include <rtt/Command.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Property.hpp>
#include <rtt/marsh/PropertyMarshaller.hpp>
#include "PowerCubeCtrl.h"

#include <iostream>
using namespace std;


class PowerCubeCtrl_OROCOS : public RTT::TaskContext
{
	public:

		PowerCubeCtrl_OROCOS(std::string name, std::string file, int dof);
		//PowerCubeCtrl_OROCOS(std::string name);
		~PowerCubeCtrl_OROCOS();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook(){}

		/* The dataports have the following names:
		"in_Angles"
		"in_Velocities"
		"in_Currents"
		"out_Angles"
		"out_Velocities"
		*/

	protected:

		bool stopArm();
		bool isArmStopped();
		bool moveJointSpace(std::vector<double> target);
		//bool m_stopped;

		std::string m_xmlFile;
		int m_dof;

		RTT::ReadDataPort< std::vector<double> > m_in_Angles;
		bool m_in_Angles_connected;

		RTT::ReadDataPort< std::vector<double> > m_in_Velocities; // MoveVel
		bool m_in_Velocities_connected;

		RTT::ReadDataPort< std::vector<double> > m_in_Currents;
		bool m_in_Currents_connected;

		RTT::WriteDataPort< std::vector<double> > m_out_Angles; // getConfig
		RTT::WriteDataPort< std::vector<double> > m_out_Velocities; // getJointVelocities
		//WriteDataPort< std::vector<double> > m_out_Currents;

		RTT::Method<bool(void)> m_stop_method;
		//RTT::Method<bool(std::vector<double>)> m_moveJointSpace_method;

		RTT::Property<int> m_CanDev_prop;
		RTT::Property<int> m_CanBaud_prop;
		RTT::Property<int> m_dof_prop;
		RTT::Property<double> m_MaxVel_prop;
		RTT::Property<double> m_MaxAcc_prop;

		std::vector< RTT::PropertyBag > m_mod_params_bags;
		std::vector< RTT::Property<RTT::PropertyBag> > m_mod_params;

		std::vector< RTT::Property<int> > m_modId_props;
		std::vector< RTT::Property<double> > m_ul_props;
		std::vector< RTT::Property<double> > m_ll_props;
		std::vector< RTT::Property<double> > m_offset_props;

		PowerCubeCtrl m_powercubectrl;

};

#endif

