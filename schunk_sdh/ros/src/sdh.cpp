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
 *   ROS package name: schunk_sdh
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Jan 2010
 *
 * \brief
 *   Implementation of ROS node for sdh.
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

//##################
//#### includes ####

// standard includes
#include <unistd.h>

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>

// ROS message includes
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <schunk_sdh/TactileSensor.h>
#include <schunk_sdh/TactileMatrix.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// ROS diagnostic msgs
#include <diagnostic_msgs/DiagnosticArray.h>

// external includes
#include <schunk_sdh/sdh.h>
#include <schunk_sdh/dsa.h>

/*!
* \brief Implementation of ROS node for sdh.
*
* Offers actionlib and direct command interface.
*/
class SdhNode
{
	public:
		/// create a handle for this node, initialize node
		ros::NodeHandle nh_;
	private:
		// declaration of topics to publish
		ros::Publisher topicPub_JointState_;
		ros::Publisher topicPub_ControllerState_;
		ros::Publisher topicPub_TactileSensor_;
		ros::Publisher topicPub_Diagnostics_;
		
		// topic subscribers
		ros::Subscriber subSetVelocitiesRaw_;
		ros::Subscriber subSetVelocities_;

		// service servers
		ros::ServiceServer srvServer_Init_;
		ros::ServiceServer srvServer_Stop_;
		ros::ServiceServer srvServer_Recover_;
		ros::ServiceServer srvServer_SetOperationMode_;

		// actionlib server
		//actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
		std::string action_name_;

		// service clients
		//--

		// other variables
		SDH::cSDH *sdh_;
		SDH::cDSA *dsa_;
		std::vector<SDH::cSDH::eAxisState> state_;

		std::string sdhdevicetype_;
		std::string sdhdevicestring_;
		int sdhdevicenum_;
		std::string dsadevicestring_;
		int dsadevicenum_;
		int baudrate_, id_read_, id_write_;
		double timeout_;

		bool isInitialized_;
		bool isDSAInitialized_;
		bool isError_;
		int DOF_;
		double pi_;
		
		trajectory_msgs::JointTrajectory traj_;
		
		std::vector<std::string> joint_names_;
		std::vector<int> axes_;
		std::vector<double> targetAngles_; // in degrees
		std::vector<double> velocities_; // in rad/s
		bool hasNewGoal_;
		std::string operationMode_; 
		
	public:
		/*!
		* \brief Constructor for SdhNode class
		*
		* \param name Name for the actionlib server
		*/
		SdhNode(std::string name):
			as_(nh_, name, boost::bind(&SdhNode::executeCB, this, _1),true),
			action_name_(name)
		{
			pi_ = 3.1415926;
			
			nh_ = ros::NodeHandle ("~");
			isError_ = false;
			// diagnostics
			topicPub_Diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

		}

		/*!
		* \brief Destructor for SdhNode class
		*/
		~SdhNode() 
		{
			if(isDSAInitialized_)
				dsa_->Close();
			if(isInitialized_)
				sdh_->Close();
			delete sdh_;
		}


		/*!
		* \brief Initializes node to get parameters, subscribe and publish to topics.
		*/
		bool init()
		{
			// initialize member variables
			isInitialized_ = false;
			isDSAInitialized_ = false;
			hasNewGoal_ = false;

			// implementation of topics to publish
			topicPub_JointState_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
			topicPub_ControllerState_ = nh_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 1);
			topicPub_TactileSensor_ = nh_.advertise<schunk_sdh::TactileSensor>("tactile_data", 1);

			// pointer to sdh
			sdh_ = new SDH::cSDH(false, false, 0); //(_use_radians=false, bool _use_fahrenheit=false, int _debug_level=0)

			// implementation of service servers
			srvServer_Init_ = nh_.advertiseService("init", &SdhNode::srvCallback_Init, this);
			srvServer_Stop_ = nh_.advertiseService("stop", &SdhNode::srvCallback_Stop, this);
			srvServer_Recover_ = nh_.advertiseService("recover", &SdhNode::srvCallback_Init, this); //HACK: There is no recover implemented yet, so we execute a init
			srvServer_SetOperationMode_ = nh_.advertiseService("set_operation_mode", &SdhNode::srvCallback_SetOperationMode, this);
			
			subSetVelocitiesRaw_ = nh_.subscribe("set_velocities_raw", 1, &SdhNode::topicCallback_setVelocitiesRaw, this);
			subSetVelocities_ = nh_.subscribe("set_velocities", 1, &SdhNode::topicCallback_setVelocities, this);

			// getting hardware parameters from parameter server
			nh_.param("sdhdevicetype", sdhdevicetype_, std::string("PCAN"));
			nh_.param("sdhdevicestring", sdhdevicestring_, std::string("/dev/pcan0"));
			nh_.param("sdhdevicenum", sdhdevicenum_, 0);
			
			nh_.param("dsadevicestring", dsadevicestring_, std::string(""));
			nh_.param("dsadevicenum", dsadevicenum_, 0);
			
			nh_.param("baudrate", baudrate_, 1000000);
			nh_.param("timeout", timeout_, (double)0.04);
			nh_.param("id_read", id_read_, 43);
			nh_.param("id_write", id_write_, 42);

			// get joint_names from parameter server
			ROS_INFO("getting joint_names from parameter server");
			XmlRpc::XmlRpcValue joint_names_param;
			if (nh_.hasParam("joint_names"))
			{
				nh_.getParam("joint_names", joint_names_param);
			}
			else
			{
				ROS_ERROR("Parameter joint_names not set, shutting down node...");
				nh_.shutdown();
				return false;
			}
			DOF_ = joint_names_param.size();
			joint_names_.resize(DOF_);
			for (int i = 0; i<DOF_; i++ )
			{
				joint_names_[i] = (std::string)joint_names_param[i];
			}
			std::cout << "joint_names = " << joint_names_param << std::endl;
			
			// define axes to send to sdh
			axes_.resize(DOF_);
			velocities_.resize(DOF_);
			for(int i=0; i<DOF_; i++)
			{
				axes_[i] = i;
			}
			ROS_INFO("DOF = %d",DOF_);
			
			state_.resize(axes_.size());

			nh_.param("OperationMode", operationMode_, std::string("position"));
			return true;
		}

		/*!
		* \brief Executes the callback from the actionlib
		*
		* Set the current goal to aborted after receiving a new goal and write new goal to a member variable. Wait for the goal to finish and set actionlib status to succeeded.
		* \param goal JointTrajectoryGoal
		*/
		//void executeCB(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal)
		void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
		{			
			ROS_INFO("sdh: executeCB");
			if (operationMode_ != "position")
			{
				ROS_ERROR("%s: Rejected, sdh not in position mode", action_name_.c_str());
				as_.setAborted();
				return;
			}
			if (!isInitialized_)
			{
				ROS_ERROR("%s: Rejected, sdh not initialized", action_name_.c_str());
				as_.setAborted();
				return;
			}

			if (goal->trajectory.points.empty() || goal->trajectory.points[0].positions.size() != size_t(DOF_))
			{
				ROS_ERROR("%s: Rejected, malformed FollowJointTrajectoryGoal", action_name_.c_str());
				as_.setAborted();
				return;
			}
			while (hasNewGoal_ == true ) usleep(10000);

			std::map<std::string,int> dict;
			for (int idx=0; idx<goal->trajectory.joint_names.size(); idx++)
			{
				dict[goal->trajectory.joint_names[idx]] = idx;
			}

			targetAngles_.resize(DOF_);
			targetAngles_[0] = goal->trajectory.points[0].positions[dict["sdh_knuckle_joint"]]*180.0/pi_; // sdh_knuckle_joint
			targetAngles_[1] = goal->trajectory.points[0].positions[dict["sdh_finger_22_joint"]]*180.0/pi_; // sdh_finger22_joint
			targetAngles_[2] = goal->trajectory.points[0].positions[dict["sdh_finger_23_joint"]]*180.0/pi_; // sdh_finger23_joint
			targetAngles_[3] = goal->trajectory.points[0].positions[dict["sdh_thumb_2_joint"]]*180.0/pi_; // sdh_thumb2_joint
			targetAngles_[4] = goal->trajectory.points[0].positions[dict["sdh_thumb_3_joint"]]*180.0/pi_; // sdh_thumb3_joint
			targetAngles_[5] = goal->trajectory.points[0].positions[dict["sdh_finger_12_joint"]]*180.0/pi_; // sdh_finger12_joint
			targetAngles_[6] = goal->trajectory.points[0].positions[dict["sdh_finger_13_joint"]]*180.0/pi_; // sdh_finger13_joint
			ROS_INFO("received position goal: [['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']] = [%f,%f,%f,%f,%f,%f,%f]",goal->trajectory.points[0].positions[dict["sdh_knuckle_joint"]],goal->trajectory.points[0].positions[dict["sdh_thumb_2_joint"]],goal->trajectory.points[0].positions[dict["sdh_thumb_3_joint"]],goal->trajectory.points[0].positions[dict["sdh_finger_12_joint"]],goal->trajectory.points[0].positions[dict["sdh_finger_13_joint"]],goal->trajectory.points[0].positions[dict["sdh_finger_22_joint"]],goal->trajectory.points[0].positions[dict["sdh_finger_23_joint"]]);
		
			hasNewGoal_ = true;
			
			usleep(500000); // needed sleep until sdh starts to change status from idle to moving
			
			bool finished = false;
			while(finished == false)
			{
				if (as_.isNewGoalAvailable())
				{
					ROS_WARN("%s: Aborted", action_name_.c_str());
					as_.setAborted();
					return;
				}
				for ( unsigned int i = 0; i < state_.size(); i++ )
		 		{
		 			ROS_DEBUG("state[%d] = %d",i,state_[i]);
		 			if (state_[i] == 0)
		 			{
		 				finished = true;
		 			}
		 			else
		 			{	
		 				finished = false;
		 			}
		 		}
		 		usleep(10000);
				//feedback_ = 
				//as_.send feedback_
			}

			// set the action state to succeeded			
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			//result_.result.data = "succesfully received new goal";
			//result_.success = 1;
			//as_.setSucceeded(result_);
			as_.setSucceeded();
		}

		void topicCallback_setVelocitiesRaw(const std_msgs::Float32MultiArrayPtr& velocities)
		{
			if (!isInitialized_)
			{
				ROS_ERROR("%s: Rejected, sdh not initialized", action_name_.c_str());
				return;
			}
			if(velocities->data.size() != velocities_.size()){
				ROS_ERROR("Velocity array dimension mismatch");
				return;
			}
			if (operationMode_ != "velocity")
			{
				ROS_ERROR("%s: Rejected, sdh not in velocity mode", action_name_.c_str());
				return;
			}

			// TODO: write proper lock!
			while (hasNewGoal_ == true ) usleep(10000);

			velocities_[0] = velocities->data[0] * 180.0 / pi_; // sdh_knuckle_joint
			velocities_[1] = velocities->data[5] * 180.0 / pi_; // sdh_finger22_joint
			velocities_[2] = velocities->data[6] * 180.0 / pi_; // sdh_finger23_joint
			velocities_[3] = velocities->data[1] * 180.0 / pi_; // sdh_thumb2_joint
			velocities_[4] = velocities->data[2] * 180.0 / pi_; // sdh_thumb3_joint
			velocities_[5] = velocities->data[3] * 180.0 / pi_; // sdh_finger12_joint
			velocities_[6] = velocities->data[4] * 180.0 / pi_; // sdh_finger13_joint

			hasNewGoal_ = true;
		}
 		bool parseDegFromJointValue(const brics_actuator::JointValue& val, double &deg_val){
		    if (val.unit == "rad/s"){
			deg_val = val.value  * 180.0 / pi_;
			return true;
		    }else if (val.unit == "deg/s"){
			deg_val = val.value;
			return true;
		    }else {
			ROS_ERROR_STREAM("Rejected message, unit '" << val.unit << "' not supported");
			return false;
		    }
		}
		void topicCallback_setVelocities(const brics_actuator::JointVelocities::ConstPtr& msg)
		{
			if (!isInitialized_)
			{
				ROS_ERROR("%s: Rejected, sdh not initialized", action_name_.c_str());
				return;
			}
			if(msg->velocities.size() != velocities_.size()){
				ROS_ERROR("Velocity array dimension mismatch");
				return;
			}
			if (operationMode_ != "velocity")
			{
				ROS_ERROR("%s: Rejected, sdh not in velocity mode", action_name_.c_str());
				return;
			}

			// TODO: write proper lock!
			while (hasNewGoal_ == true ) usleep(10000);
			bool valid = true;

			valid = valid && parseDegFromJointValue(msg->velocities[0], velocities_[0]); // sdh_knuckle_joint
			valid = valid && parseDegFromJointValue(msg->velocities[5], velocities_[1]); // sdh_finger22_joint
			valid = valid && parseDegFromJointValue(msg->velocities[6], velocities_[2]); // sdh_finger23_joint
			valid = valid && parseDegFromJointValue(msg->velocities[1], velocities_[3]); // sdh_thumb2_joint
			valid = valid && parseDegFromJointValue(msg->velocities[2], velocities_[4]); // sdh_thumb3_joint
			valid = valid && parseDegFromJointValue(msg->velocities[3], velocities_[5]); // sdh_finger12_joint
			valid = valid && parseDegFromJointValue(msg->velocities[4], velocities_[6]); // sdh_finger13_joint

			if (valid) hasNewGoal_ = true;
		}		
		/*!
		* \brief Executes the service callback for init.
		*
		* Connects to the hardware and initialized it.
		* \param req Service request
		* \param res Service response
		*/
		bool srvCallback_Init(cob_srvs::Trigger::Request &req,
							cob_srvs::Trigger::Response &res )
		{

			if (isInitialized_ == false)
			{
				//Init Hand connection	
				
				try
				{
					if(sdhdevicetype_.compare("RS232")==0)
					{
						sdh_->OpenRS232( sdhdevicenum_, 115200, 1, sdhdevicestring_.c_str());
						ROS_INFO("Initialized RS232 for SDH");
						isInitialized_ = true;
					}
					if(sdhdevicetype_.compare("PCAN")==0)
					{
						ROS_INFO("Starting initializing PEAKCAN");
						sdh_->OpenCAN_PEAK(baudrate_, timeout_, id_read_, id_write_, sdhdevicestring_.c_str());
						ROS_INFO("Initialized PEAK CAN for SDH");
						isInitialized_ = true;
					}
					if(sdhdevicetype_.compare("ESD")==0)
					{
						ROS_INFO("Starting initializing ESD");
						if(strcmp(sdhdevicestring_.c_str(), "/dev/can0") == 0)
						{
							ROS_INFO("Initializing ESD on device %s",sdhdevicestring_.c_str());
							sdh_->OpenCAN_ESD(0, baudrate_, timeout_, id_read_, id_write_ );
						}
						else if(strcmp(sdhdevicestring_.c_str(), "/dev/can1") == 0)
						{
							ROS_INFO("Initializin ESD on device %s",sdhdevicestring_.c_str());
							sdh_->OpenCAN_ESD(1, baudrate_, timeout_, id_read_, id_write_ );
						}
						else
						{
							ROS_ERROR("Currently only support for /dev/can0 and /dev/can1");
							res.success.data = false;
							res.error_message.data = "Currently only support for /dev/can0 and /dev/can1";
							return true;
						}
						ROS_INFO("Initialized ESDCAN for SDH");	
						isInitialized_ = true;
					}
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					res.success.data = false;
					res.error_message.data = e->what();
					delete e;
					return true;
				}
				
				//Init tactile data
				if(!dsadevicestring_.empty())  {
					try
					{
						dsa_ = new SDH::cDSA(0, dsadevicenum_, dsadevicestring_.c_str());
						//dsa_->SetFramerate( 0, true, false );
						dsa_->SetFramerate( 1, true );
						ROS_INFO("Initialized RS232 for DSA Tactile Sensors on device %s",dsadevicestring_.c_str());
						// ROS_INFO("Set sensitivity to 1.0");
						// for(int i=0; i<6; i++)
						// 	dsa_->SetMatrixSensitivity(i, 1.0);
						isDSAInitialized_ = true;
					}
					catch (SDH::cSDHLibraryException* e)
					{
						isDSAInitialized_ = false;
						ROS_ERROR("An exception was caught: %s", e->what());
						res.success.data = false;
						res.error_message.data = e->what();
						delete e;
						return true;
					}
				}
			}
			else
			{
				ROS_WARN("...sdh already initialized...");
				res.success.data = true;
				res.error_message.data = "sdh already initialized";
			}
			
			res.success.data = true;
			return true;
		}

		/*!
		* \brief Executes the service callback for stop.
		*
		* Stops all hardware movements.
		* \param req Service request
		* \param res Service response
		*/
		bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
							cob_srvs::Trigger::Response &res )
		{
			ROS_INFO("Stopping sdh");

			// stopping all arm movements
			try
			{
				sdh_->Stop();
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}

		ROS_INFO("Stopping sdh succesfull");
		res.success.data = true;
		return true;
	}

	/*!
	* \brief Executes the service callback for recover.
	*
	* Recovers the hardware after an emergency stop.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_Recover(cob_srvs::Trigger::Request &req,
							cob_srvs::Trigger::Response &res )
	{
		ROS_WARN("Service recover not implemented yet");
		res.success.data = true;
		res.error_message.data = "Service recover not implemented yet";
		return true;
	}
	
	/*!
	* \brief Executes the service callback for set_operation_mode.
	*
	* Changes the operation mode.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req,
									cob_srvs::SetOperationMode::Response &res )
	{
		hasNewGoal_ = false;
		sdh_->Stop();
		ROS_INFO("Set operation mode to [%s]", req.operation_mode.data.c_str());
		operationMode_ = req.operation_mode.data;
		res.success.data = true;
		if( operationMode_ == "position"){
			sdh_->SetController(SDH::cSDH::eCT_POSE);
		}else if( operationMode_ == "velocity"){
			try{
				sdh_->SetController(SDH::cSDH::eCT_VELOCITY);
				sdh_->SetAxisEnable(sdh_->All, 1.0);
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
		}else{
			ROS_ERROR_STREAM("Operation mode '" << req.operation_mode.data << "'  not supported");
		}
		return true;
	}

	/*!
	* \brief Main routine to update sdh.
	*
	* Sends target to hardware and reads out current configuration.
	*/
	void updateSdh()
	{
		ROS_DEBUG("updateJointState");
		if (isInitialized_ == true)
		{
			if (hasNewGoal_ == true)
			{
				// stop sdh first when new goal arrived
				try
				{
					sdh_->Stop();
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
		
				if (operationMode_ == "position")
				{
					ROS_DEBUG("moving sdh in position mode");

					try
					{
						sdh_->SetAxisTargetAngle( axes_, targetAngles_ );
						sdh_->MoveHand(false);
					}
					catch (SDH::cSDHLibraryException* e)
					{
						ROS_ERROR("An exception was caught: %s", e->what());
						delete e;
					}
				}
				else if (operationMode_ == "velocity")
				{
					ROS_DEBUG("moving sdh in velocity mode");
					try
					{
						sdh_->SetAxisTargetVelocity(axes_,velocities_);
						// ROS_DEBUG_STREAM("velocities: " << velocities_[0] << " "<< velocities_[1] << " "<< velocities_[2] << " "<< velocities_[3] << " "<< velocities_[4] << " "<< velocities_[5] << " "<< velocities_[6]);
					}
					catch (SDH::cSDHLibraryException* e)
					{
						ROS_ERROR("An exception was caught: %s", e->what());
						delete e;
					}
				}
				else if (operationMode_ == "effort")
				{
					ROS_DEBUG("moving sdh in effort mode");
					//sdh_->MoveVel(goal->trajectory.points[0].velocities);
					ROS_WARN("Moving in effort mode currently disabled");
				}
				else
				{
					ROS_ERROR("sdh neither in position nor in velocity nor in effort mode. OperationMode = [%s]", operationMode_.c_str());
				}
				
				hasNewGoal_ = false;
			}

	 		// read and publish joint angles and velocities
			std::vector<double> actualAngles;
			try
			{
				actualAngles = sdh_->GetAxisActualAngle( axes_ );
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
			std::vector<double> actualVelocities;
			try
			{
				actualVelocities = sdh_->GetAxisActualVelocity( axes_ );
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
			
			ROS_DEBUG("received %d angles from sdh",(int)actualAngles.size());
			
			ros::Time time = ros::Time::now();
			
			// create joint_state message
			sensor_msgs::JointState msg;
			msg.header.stamp = time;
			msg.name.resize(DOF_);
			msg.position.resize(DOF_);
			msg.velocity.resize(DOF_);
			msg.effort.resize(DOF_);
			// set joint names and map them to angles
			msg.name = joint_names_;
			//['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
			// pos
			msg.position[0] = actualAngles[0]*pi_/180.0; // sdh_knuckle_joint
			msg.position[1] = actualAngles[3]*pi_/180.0; // sdh_thumb_2_joint
			msg.position[2] = actualAngles[4]*pi_/180.0; // sdh_thumb_3_joint
			msg.position[3] = actualAngles[5]*pi_/180.0; // sdh_finger_12_joint
			msg.position[4] = actualAngles[6]*pi_/180.0; // sdh_finger_13_joint
			msg.position[5] = actualAngles[1]*pi_/180.0; // sdh_finger_22_joint
			msg.position[6] = actualAngles[2]*pi_/180.0; // sdh_finger_23_joint
			// vel			
			msg.velocity[0] = actualVelocities[0]*pi_/180.0; // sdh_knuckle_joint
			msg.velocity[1] = actualVelocities[3]*pi_/180.0; // sdh_thumb_2_joint
			msg.velocity[2] = actualVelocities[4]*pi_/180.0; // sdh_thumb_3_joint
			msg.velocity[3] = actualVelocities[5]*pi_/180.0; // sdh_finger_12_joint
			msg.velocity[4] = actualVelocities[6]*pi_/180.0; // sdh_finger_13_joint
			msg.velocity[5] = actualVelocities[1]*pi_/180.0; // sdh_finger_22_joint
			msg.velocity[6] = actualVelocities[2]*pi_/180.0; // sdh_finger_23_joint
			// publish message
			topicPub_JointState_.publish(msg);
			
			
			// because the robot_state_publisher doen't know about the mimic joint, we have to publish the coupled joint separately
			sensor_msgs::JointState  mimicjointmsg;
			mimicjointmsg.header.stamp = time;
			mimicjointmsg.name.resize(1);
			mimicjointmsg.position.resize(1);
			mimicjointmsg.velocity.resize(1);
			mimicjointmsg.name[0] = "sdh_finger_21_joint";
			mimicjointmsg.position[0] = msg.position[0]; // sdh_knuckle_joint = sdh_finger_21_joint
			mimicjointmsg.velocity[0] = msg.velocity[0]; // sdh_knuckle_joint = sdh_finger_21_joint
			topicPub_JointState_.publish(mimicjointmsg);
			
			
			// publish controller state message
			pr2_controllers_msgs::JointTrajectoryControllerState controllermsg;
			controllermsg.header.stamp = time;
			controllermsg.joint_names.resize(DOF_);
			controllermsg.desired.positions.resize(DOF_);
			controllermsg.desired.velocities.resize(DOF_);
			controllermsg.actual.positions.resize(DOF_);
			controllermsg.actual.velocities.resize(DOF_);
			controllermsg.error.positions.resize(DOF_);
			controllermsg.error.velocities.resize(DOF_);
			// set joint names and map them to angles
			controllermsg.joint_names = joint_names_;
			//['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
			// desired pos
			if (targetAngles_.size() != 0)
			{
				controllermsg.desired.positions[0] = targetAngles_[0]*pi_/180.0; // sdh_knuckle_joint
				controllermsg.desired.positions[1] = targetAngles_[3]*pi_/180.0; // sdh_thumb_2_joint
				controllermsg.desired.positions[2] = targetAngles_[4]*pi_/180.0; // sdh_thumb_3_joint
				controllermsg.desired.positions[3] = targetAngles_[5]*pi_/180.0; // sdh_finger_12_joint
				controllermsg.desired.positions[4] = targetAngles_[6]*pi_/180.0; // sdh_finger_13_joint
				controllermsg.desired.positions[5] = targetAngles_[1]*pi_/180.0; // sdh_finger_22_joint
				controllermsg.desired.positions[6] = targetAngles_[2]*pi_/180.0; // sdh_finger_23_joint
			}
			// desired vel
				// they are all zero
			// actual pos			
			controllermsg.actual.positions = msg.position;
			// actual vel
			controllermsg.actual.velocities = msg.velocity;
			// error, calculated out of desired and actual values
			for (int i = 0; i<DOF_; i++ )
			{
				controllermsg.error.positions[i] = controllermsg.desired.positions[i] - controllermsg.actual.positions[i];
				controllermsg.error.velocities[i] = controllermsg.desired.velocities[i] - controllermsg.actual.velocities[i];
			}
			// publish controller message
			topicPub_ControllerState_.publish(controllermsg);

			// read sdh status
			state_ = sdh_->GetAxisActualState(axes_);
		}
		else
		{
			ROS_DEBUG("sdh not initialized");
		}
		// publishing diagnotic messages
	    diagnostic_msgs::DiagnosticArray diagnostics;
	    diagnostics.status.resize(1);
	    // set data to diagnostics
	    if(isError_)
	    {
	      diagnostics.status[0].level = 2;
	      diagnostics.status[0].name = "schunk_powercube_chain";
	      diagnostics.status[0].message = "one or more drives are in Error mode";
	    }
	    else
	    {
	      if (isInitialized_)
	      {
	        diagnostics.status[0].level = 0;
	        diagnostics.status[0].name = nh_.getNamespace(); //"schunk_powercube_chain";
	        if(isDSAInitialized_)
	        	diagnostics.status[0].message = "sdh with tactile sensing initialized and running";
	        else
	        	diagnostics.status[0].message = "sdh initialized and running, tactile sensors not connected";	
	      }
	      else
	      {
	        diagnostics.status[0].level = 1;
	        diagnostics.status[0].name = nh_.getNamespace(); //"schunk_powercube_chain";
	        diagnostics.status[0].message = "sdh not initialized";
	      }
	    }
	    // publish diagnostic message
	    topicPub_Diagnostics_.publish(diagnostics);

	}

	/*!
	* \brief Main routine to update dsa.
	*
	* Reads out current values.
	*/
	void updateDsa()
	{
                static const int dsa_reorder[6] = { 2 ,3, 4, 5, 0 , 1 }; // t1,t2,f11,f12,f21,f22
		ROS_DEBUG("updateTactileData");

		if(isDSAInitialized_)
		{
			// read tactile data
			for(int i=0; i<7; i++)
			{
				try
				{
					//dsa_->SetFramerate( 0, true, true );
					dsa_->UpdateFrame();
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
			}

			schunk_sdh::TactileSensor msg;
			msg.header.stamp = ros::Time::now();
			int m, x, y;
			msg.tactile_matrix.resize(dsa_->GetSensorInfo().nb_matrices);
			for ( int i = 0; i < dsa_->GetSensorInfo().nb_matrices; i++ )
			{
				m = dsa_reorder[i];                                  
				schunk_sdh::TactileMatrix &tm = msg.tactile_matrix[i];
				tm.matrix_id = i;
				tm.cells_x = dsa_->GetMatrixInfo( m ).cells_x;
				tm.cells_y = dsa_->GetMatrixInfo( m ).cells_y;
				tm.tactile_array.resize(tm.cells_x * tm.cells_y);
				for ( y = 0; y < tm.cells_y; y++ )
				{
					for ( x = 0; x < tm.cells_x; x++ )
						tm.tactile_array[tm.cells_x*y + x] = dsa_->GetTexel( m, x, y );
				}
			}
			//publish matrix
			topicPub_TactileSensor_.publish(msg);
		}
	}	 
}; //SdhNode

/*!
* \brief Main loop of ROS node.
*
* Running with a specific frequency defined by loop_rate.
*/
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "schunk_sdh");

	//SdhNode sdh_node(ros::this_node::getName() + "/joint_trajectory_action");
	SdhNode sdh_node(ros::this_node::getName() + "/follow_joint_trajectory");
	if (!sdh_node.init()) return 0;
	
	ROS_INFO("...sdh node running...");

	double frequency;
	if (sdh_node.nh_.hasParam("frequency"))
	{
		sdh_node.nh_.getParam("frequency", frequency);
	}
	else
	{
		frequency = 5; //Hz
		ROS_WARN("Parameter frequency not available, setting to default value: %f Hz", frequency);
	}

	//sleep(1);
	ros::Rate loop_rate(frequency); // Hz
	while(sdh_node.nh_.ok())
	{
		// publish JointState
		sdh_node.updateSdh();
		
		// publish TactileData
		sdh_node.updateDsa();
		
		// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

