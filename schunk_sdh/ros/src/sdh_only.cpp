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
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

// ROS service includes
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetString.h>

// ROS diagnostic msgs
#include <diagnostic_msgs/DiagnosticArray.h>

// external includes
#include <schunk_sdh/sdh.h>

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
		ros::NodeHandle nh_private_;
		
	private:
		// declaration of topics to publish
		ros::Publisher topicPub_JointState_;
		ros::Publisher topicPub_ControllerState_;
		ros::Publisher topicPub_Diagnostics_;
		
		// topic subscribers
		ros::Subscriber subSetVelocitiesRaw_;

		// service servers
		ros::ServiceServer srvServer_Init_;
		ros::ServiceServer srvServer_Stop_;
		ros::ServiceServer srvServer_Recover_;
		ros::ServiceServer srvServer_SetOperationMode_;

		// actionlib server
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
		std::string action_name_;

		// service clients
		//--

		// other variables
		SDH::cSDH *sdh_;
		std::vector<SDH::cSDH::eAxisState> state_;

		std::string sdhdevicetype_;
		std::string sdhdevicestring_;
		int sdhdevicenum_;
		int baudrate_, id_read_, id_write_;
		double timeout_;

		bool isInitialized_;
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
		SdhNode():
			as_(nh_, "joint_trajectory_controller/follow_joint_trajectory", boost::bind(&SdhNode::executeCB, this, _1),true),
			action_name_("follow_joint_trajectory")
		{
			nh_private_ = ros::NodeHandle ("~");
			pi_ = 3.1415926;
			isError_ = false;
		}

		/*!
		* \brief Destructor for SdhNode class
		*/
		~SdhNode() 
		{
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
			hasNewGoal_ = false;

			// implementation of topics to publish
			topicPub_JointState_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
			topicPub_ControllerState_ = nh_.advertise<control_msgs::JointTrajectoryControllerState>("joint_trajectory_controller/state", 1);
			topicPub_Diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);

			// pointer to sdh
			sdh_ = new SDH::cSDH(false, false, 0); //(_use_radians=false, bool _use_fahrenheit=false, int _debug_level=0)

			// implementation of service servers
			srvServer_Init_ = nh_.advertiseService("driver/init", &SdhNode::srvCallback_Init, this);
			srvServer_Stop_ = nh_.advertiseService("driver/stop", &SdhNode::srvCallback_Stop, this);
			srvServer_Recover_ = nh_.advertiseService("driver/recover", &SdhNode::srvCallback_Init, this); //HACK: There is no recover implemented yet, so we execute a init
			srvServer_SetOperationMode_ = nh_.advertiseService("driver/set_operation_mode", &SdhNode::srvCallback_SetOperationMode, this);
			
			subSetVelocitiesRaw_ = nh_.subscribe("joint_group_velocity_controller/command", 1, &SdhNode::topicCallback_setVelocitiesRaw, this);
			
			// getting hardware parameters from parameter server
			nh_private_.param("sdhdevicetype", sdhdevicetype_, std::string("PCAN"));
			nh_private_.param("sdhdevicestring", sdhdevicestring_, std::string("/dev/pcan0"));
			nh_private_.param("sdhdevicenum", sdhdevicenum_, 0);
			
			nh_private_.param("baudrate", baudrate_, 1000000);
			nh_private_.param("timeout", timeout_, (double)0.04);
			nh_private_.param("id_read", id_read_, 43);
			nh_private_.param("id_write", id_write_, 42);

			// get joint_names from parameter server
			ROS_INFO("getting joint_names from parameter server");
			XmlRpc::XmlRpcValue joint_names_param;
			if (nh_private_.hasParam("joint_names"))
			{
				nh_private_.getParam("joint_names", joint_names_param);
			}
			else
			{
				ROS_ERROR("Parameter 'joint_names' not set, shutting down node...");
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
			
			nh_private_.param("OperationMode", operationMode_, std::string("position"));
			return true;
		}
		/*!
		* \brief Switches operation mode if possible
		*
		* \param mode new mode
		*/
		bool switchOperationMode(const std::string &mode){
			hasNewGoal_ = false;
			sdh_->Stop();
			
			try{
				if(mode == "position"){
					sdh_->SetController(SDH::cSDH::eCT_POSE);
				}else if(mode == "velocity"){
					sdh_->SetController(SDH::cSDH::eCT_VELOCITY);
				}else{
					ROS_ERROR_STREAM("Operation mode '" << mode << "'  not supported");
					return false;
				}
				sdh_->SetAxisEnable(sdh_->All, 1.0); // TODO: check if necessary
			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
				return false;
			}
			
			operationMode_ = mode;
			return true;

		}

		/*!
		* \brief Executes the callback from the actionlib
		*
		* Set the current goal to aborted after receiving a new goal and write new goal to a member variable. Wait for the goal to finish and set actionlib status to succeeded.
		* \param goal JointTrajectoryGoal
		*/
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
			}

			// set the action state to succeeded
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			as_.setSucceeded();
		}

		void topicCallback_setVelocitiesRaw(const std_msgs::Float64MultiArrayPtr& velocities)
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

		/*!
		* \brief Executes the service callback for init.
		*
		* Connects to the hardware and initialized it.
		* \param req Service request
		* \param res Service response
		*/
		bool srvCallback_Init(std_srvs::Trigger::Request &req,
							std_srvs::Trigger::Response &res )
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
							res.success = false;
							res.message = "Currently only support for /dev/can0 and /dev/can1";
							return true;
						}
						ROS_INFO("Initialized ESDCAN for SDH");	
						isInitialized_ = true;
					}
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					res.success = false;
					res.message = e->what();
					delete e;
					return true;
				}
				if(!switchOperationMode(operationMode_)){
					res.success = false;
					res.message = "Could not set operation mode to '" + operationMode_ + "'";
					return true;
				}
			}
			else
			{
				ROS_WARN("...sdh already initialized...");
				res.success = true;
				res.message = "sdh already initialized";
			}
			
			res.success = true;
			return true;
		}

		/*!
		* \brief Executes the service callback for stop.
		*
		* Stops all hardware movements.
		* \param req Service request
		* \param res Service response
		*/
		bool srvCallback_Stop(std_srvs::Trigger::Request &req,
							std_srvs::Trigger::Response &res )
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
		res.success = true;
		return true;
	}

	/*!
	* \brief Executes the service callback for recover.
	*
	* Recovers the hardware after an emergency stop.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_Recover(std_srvs::Trigger::Request &req,
							std_srvs::Trigger::Response &res )
	{
		ROS_WARN("Service recover not implemented yet");
		res.success = true;
		res.message = "Service recover not implemented yet";
		return true;
	}
	
	/*!
	* \brief Executes the service callback for set_operation_mode.
	*
	* Changes the operation mode.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_SetOperationMode(cob_srvs::SetString::Request &req,
									cob_srvs::SetString::Response &res )
	{
		hasNewGoal_ = false;
		sdh_->Stop();
		res.success = switchOperationMode(req.data);
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
			ROS_ERROR_STREAM("Operation mode '" << req.data << "'  not supported");
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
			control_msgs::JointTrajectoryControllerState controllermsg;
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
	        diagnostics.status[0].message = "sdh initialized and running";	
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

	SdhNode sdh_node;
	if (!sdh_node.init()) return 0;
	
	ROS_INFO("...sdh node running...");

	double frequency;
	if (sdh_node.nh_private_.hasParam("frequency"))
	{
		sdh_node.nh_private_.getParam("frequency", frequency);
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
		
		// sleep and waiting for messages, callbacks
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

