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
 *   Implementation of ROS node for powercube_chain.
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
//##################

// standard includes
// --

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>

// ROS message includes
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// ROS service includes
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetString.h>

// own includes
#include <schunk_powercube_chain/PowerCubeCtrl.h>
#include <schunk_powercube_chain/PowerCubeCtrlParams.h>
//#include <schunk_powercube_chain/Diagnostics.h>

/*!
 * \brief Implementation of ROS node for powercube_chain.
 *
 * Offers velocity and position interface.
 */
class PowerCubeChainNode
{
public:
  /// create a handle for this node, initialize node
  ros::NodeHandle n_;
  ros::NodeHandle n_private_;

  /// declaration of topics to publish
  ros::Publisher topicPub_JointState_;
  ros::Publisher topicPub_ControllerState_;
  ros::Publisher topicPub_OperationMode_;
  ros::Publisher topicPub_Diagnostic_;

  /// declaration of topics to subscribe, callback is called for new messages arriving
  ros::Subscriber topicSub_CommandPos_;
  ros::Subscriber topicSub_CommandVel_;

  /// declaration of service servers
  ros::ServiceServer srvServer_Init_;
  ros::ServiceServer srvServer_SetOperationMode_;
  ros::ServiceServer srvServer_Stop_;
  ros::ServiceServer srvServer_Recover_;

  /// handle for powercube_chain
  PowerCubeCtrl *pc_ctrl_;

  /// handle for powercube_chain parameters
  PowerCubeCtrlParams *pc_params_;

  /// member variables
  bool initialized_;
  bool stopped_;
  bool error_;
  std::string error_msg_;
  ros::Time last_publish_time_;

  /// Constructor
  PowerCubeChainNode()
  {
    n_private_ = ros::NodeHandle("~");

    pc_params_ = new PowerCubeCtrlParams();
    pc_ctrl_ = new PowerCubeCtrl(pc_params_);

    /// implementation of topics to publish
    topicPub_JointState_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);
    topicPub_ControllerState_ =  n_.advertise<control_msgs::JointTrajectoryControllerState>("joint_trajectory_controller/state", 1);
    topicPub_Diagnostic_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);

    /// implementation of topics to subscribe
    topicSub_CommandPos_ = n_.subscribe("joint_group_position_controller/command", 1, &PowerCubeChainNode::topicCallback_CommandPos, this);
    topicSub_CommandVel_ = n_.subscribe("joint_group_velocity_controller/command", 1, &PowerCubeChainNode::topicCallback_CommandVel, this);

    /// implementation of service servers
    srvServer_Init_ = n_.advertiseService("driver/init", &PowerCubeChainNode::srvCallback_Init, this);
    srvServer_Stop_ = n_.advertiseService("driver/stop", &PowerCubeChainNode::srvCallback_Stop, this);
    srvServer_Recover_ = n_.advertiseService("driver/recover", &PowerCubeChainNode::srvCallback_Recover, this);
    srvServer_SetOperationMode_ = n_.advertiseService("driver/set_operation_mode", &PowerCubeChainNode::srvCallback_SetOperationMode, this);
    topicPub_OperationMode_ = n_.advertise<std_msgs::String>("driver/current_operationmode", 1);

    initialized_ = false;
    stopped_ = true;
    error_ = false;
    last_publish_time_ = ros::Time::now();
  }

  /// Destructor
  ~PowerCubeChainNode()
  {
    bool closed = pc_ctrl_->Close();
    if (closed)
      ROS_INFO("PowerCube Device closed!");
  }

  /*!
   * \brief Gets parameters from the ROS parameter server and configures the powercube_chain.
   */
  void getROSParameters()
  {
    /// get CanModule
    std::string CanModule;
    if (n_private_.hasParam("can_module"))
    {
      n_private_.getParam("can_module", CanModule);
    }
    else
    {
      ROS_ERROR("Parameter can_module not set, shutting down node...");
      n_.shutdown();
    }

    /// get CanDevice
    std::string CanDevice;
    if (n_private_.hasParam("can_device"))
    {
      n_private_.getParam("can_device", CanDevice);
    }
    else
    {
      ROS_ERROR("Parameter can_device not set, shutting down node...");
      n_.shutdown();
    }

    /// get CanBaudrate
    int CanBaudrate;
    if (n_private_.hasParam("can_baudrate"))
    {
      n_private_.getParam("can_baudrate", CanBaudrate);
    }
    else
    {
      ROS_ERROR("Parameter can_baudrate not set, shutting down node...");
      n_.shutdown();
    }

    /// get Modul IDs
    XmlRpc::XmlRpcValue ModulIDsXmlRpc;
    std::vector<int> ModulIDs;
    if (n_private_.hasParam("modul_ids"))
    {
      n_private_.getParam("modul_ids", ModulIDsXmlRpc);
    }
    else
    {
      ROS_ERROR("Parameter modul_ids not set, shutting down node...");
      n_.shutdown();
    }

    /// get force_use_movevel
    bool UseMoveVel;
    if (n_private_.hasParam("force_use_movevel"))
    {
      n_private_.getParam("force_use_movevel", UseMoveVel);
      ROS_INFO("Parameter force_use_movevel set, using moveVel");
    }
    else
    {
      ROS_INFO("Parameter force_use_movevel not set, using moveStep");
      UseMoveVel = false;
    }
    pc_params_->SetUseMoveVel(UseMoveVel);

    /// Resize and assign of values to the ModulIDs
    ModulIDs.resize(ModulIDsXmlRpc.size());
    for (int i = 0; i < ModulIDsXmlRpc.size(); i++)
    {
      ModulIDs[i] = (int)ModulIDsXmlRpc[i];
    }

    /// Initialize parameters
    pc_params_->Init(CanModule, CanDevice, CanBaudrate, ModulIDs);

    /// Get joint names
    XmlRpc::XmlRpcValue JointNamesXmlRpc;
    std::vector<std::string> JointNames;
    if (n_private_.hasParam("joint_names"))
    {
      n_private_.getParam("joint_names", JointNamesXmlRpc);
    }
    else
    {
      ROS_ERROR("Parameter joint_names not set, shutting down node...");
      n_.shutdown();
    }

    /// Resize and assign of values to the JointNames
    JointNames.resize(JointNamesXmlRpc.size());
    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
    {
      JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }

    /// Check dimension with with DOF
    if ((int)JointNames.size() != pc_params_->GetDOF())
    {
      ROS_ERROR("Wrong dimensions of parameter joint_names, shutting down node...");
      n_.shutdown();
    }
    pc_params_->SetJointNames(JointNames);

    /// Get max accelerations
    XmlRpc::XmlRpcValue MaxAccelerationsXmlRpc;
    std::vector<double> MaxAccelerations;
    if (n_private_.hasParam("max_accelerations"))
    {
      n_private_.getParam("max_accelerations", MaxAccelerationsXmlRpc);
    }
    else
    {
      ROS_ERROR("Parameter max_accelerations not set, shutting down node...");
      n_.shutdown();
    }

    /// Resize and assign of values to the MaxAccelerations
    MaxAccelerations.resize(MaxAccelerationsXmlRpc.size());
    for (int i = 0; i < MaxAccelerationsXmlRpc.size(); i++)
    {
      MaxAccelerations[i] = (double)MaxAccelerationsXmlRpc[i];
    }

    /// Check dimension with with DOF
    if ((int)MaxAccelerations.size() != pc_params_->GetDOF())
    {
      ROS_ERROR("Wrong dimensions of parameter max_accelerations, shutting down node...");
      n_.shutdown();
    }
    pc_params_->SetMaxAcc(MaxAccelerations);

    /// Get horizon
    double Horizon;
    if (n_private_.hasParam("horizon"))
    {
      n_private_.getParam("horizon", Horizon);
    }
    else
    {
      /// Horizon in sec
      Horizon = 0.05;
      ROS_WARN("Parameter horizon not available, setting to default value: %f sec", Horizon);
    }
    pc_ctrl_->setHorizon(Horizon);
  }

  /*!
   * \brief Gets parameters from the robot_description and configures the powercube_chain.
   */
  void getRobotDescriptionParameters()
  {
    unsigned int DOF = pc_params_->GetDOF();
    std::vector<std::string> JointNames = pc_params_->GetJointNames();

    /// Get robot_description from ROS parameter server
    std::string xml_string;
    if (n_.hasParam("/robot_description"))
    {
      n_.getParam("/robot_description", xml_string);
    }
    else
    {
      ROS_ERROR("Parameter '/robot_description' not set, shutting down node...");
      n_.shutdown();
    }

    if (xml_string.size() == 0)
    {
      ROS_ERROR("Unable to load robot model from parameter /robot_description'");
      n_.shutdown();
    }

    /// Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
      ROS_ERROR("Failed to parse urdf file");
      n_.shutdown();
    }
    ROS_DEBUG("Successfully parsed urdf file");

    /// Get max velocities out of urdf model
    std::vector<double> MaxVelocities(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      MaxVelocities[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
    }

    /// Get lower limits out of urdf model
    std::vector<double> LowerLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower;
    }

    // Get upper limits out of urdf model
    std::vector<double> UpperLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper;
    }

    /// Get offsets out of urdf model
    std::vector<double> Offsets(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
      if(model.getJoint(JointNames[i].c_str())->calibration == NULL)
        Offsets[i] = 0.0;
      else
        Offsets[i] = model.getJoint(JointNames[i].c_str())->calibration->rising.get()[0];
    }

    /// Set parameters
    pc_params_->SetMaxVel(MaxVelocities);
    pc_params_->SetLowerLimits(LowerLimits);
    pc_params_->SetUpperLimits(UpperLimits);
    pc_params_->SetOffsets(Offsets);
  }

  /*!
   * \brief Executes the callback from the command_pos topic.
   *
   * Set the current position target.
   * \param msg Float64MultiArray
   */
  void topicCallback_CommandPos(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    ROS_DEBUG("Received new position command");
    if (!initialized_)
    {
      ROS_WARN("Skipping command: powercubes not initialized");
      publishState();
      return;
    }

    if (pc_ctrl_->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK)
    {
      publishState();
      return;
    }

    PowerCubeCtrl::PC_CTRL_STATUS status;
    std::vector<std::string> errorMessages;
    pc_ctrl_->getStatus(status, errorMessages);

    /// check dimensions
    if (msg->data.size() != pc_params_->GetDOF())
    {
      ROS_ERROR("Skipping command: Commanded positionss and DOF are not same dimension.");
      return;
    }

    /// command positions to powercubes
    if (!pc_ctrl_->MoveJointSpaceSync(msg->data))
    {
      error_ = true;
      error_msg_ = pc_ctrl_->getErrorMessage();
      ROS_ERROR("Skipping command: %s", pc_ctrl_->getErrorMessage().c_str());
      return;
    }

    ROS_DEBUG("Executed position command");

    publishState();
  }

  /*!
   * \brief Executes the callback from the command_vel topic.
   *
   * Set the current velocity target.
   * \param msg Float64MultiArray
   */
  void topicCallback_CommandVel(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    ROS_DEBUG("Received new velocity command");
    if (!initialized_)
    {
      ROS_WARN("Skipping command: powercubes not initialized");
      publishState(false);
      return;
    }

    if (pc_ctrl_->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK)
    {
      publishState(false);
      return;
    }

    PowerCubeCtrl::PC_CTRL_STATUS status;
    std::vector<std::string> errorMessages;
    pc_ctrl_->getStatus(status, errorMessages);

    unsigned int DOF = pc_params_->GetDOF();

    /// check dimensions
    if (msg->data.size() != DOF)
    {
      ROS_ERROR("Skipping command: Commanded velocities and DOF are not same dimension.");
      return;
    }

    /// command velocities to powercubes
    if (!pc_ctrl_->MoveVel(msg->data))
    {
      error_ = true;
      error_msg_ = pc_ctrl_->getErrorMessage();
      ROS_ERROR("Skipping command: %s", pc_ctrl_->getErrorMessage().c_str());
      return;
    }

    ROS_DEBUG("Executed velocity command");

    publishState(false);
  }

  /*!
   * \brief Executes the service callback for init.
   *
   * Connects to the hardware and initialized it.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    if (!initialized_)
    {
      ROS_INFO("Initializing powercubes...");

      /// initialize powercubes
      if (pc_ctrl_->Init(pc_params_))
      {
        initialized_ = true;
        res.success = true;
        ROS_INFO("...initializing powercubes successful");
      }

      else
      {
        error_ = true;
        error_msg_ = pc_ctrl_->getErrorMessage();
        res.success = false;
        res.message = pc_ctrl_->getErrorMessage();
        ROS_INFO("...initializing powercubes not successful. error: %s", res.message.c_str());
      }
    }

    else
    {
      res.success = true;
      res.message = "powercubes already initialized";
      ROS_WARN("...initializing powercubes not successful. error: %s", res.message.c_str());
    }

    return true;
  }

  /*!
   * \brief Executes the service callback for stop.
   *
   * Stops all hardware movements.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Stopping powercubes...");

    /// stop powercubes
    if (pc_ctrl_->Stop())
    {
      res.success = true;
      ROS_INFO("...stopping powercubes successful.");
    }

    else
    {
      res.success = false;
      res.message = pc_ctrl_->getErrorMessage();
      ROS_ERROR("...stopping powercubes not successful. error: %s", res.message.c_str());
    }
    return true;
  }

  /*!
   * \brief Executes the service callback for recover.
   *
   * Recovers the driver after an emergency stop.
   * \param req Service request
   * \param res Service response
   */
  bool srvCallback_Recover(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Recovering powercubes...");
    if (initialized_)
    {
      /// stopping all arm movements
      if (pc_ctrl_->Recover())
      {
        error_ = false;
        error_msg_ = "";
        res.success = true;
        ROS_INFO("...recovering powercubes successful.");
      }
      else
      {
        res.success = false;
        error_ = true;
        error_msg_ = pc_ctrl_->getErrorMessage();
        res.message = pc_ctrl_->getErrorMessage();
        ROS_ERROR("...recovering powercubes not successful. error: %s", res.message.c_str());
      }
    }
    else
    {
      res.success = false;
      res.message = "powercubes not initialized";
      ROS_ERROR("...recovering powercubes not successful. error: %s", res.message.c_str());
    }

    return true;
  }

  /*!
  * \brief Executes the service callback for SetOperationMode.
  *
  * Sets the driver to different operation modes. Currently only operation_mode=velocity is supported.
  * \param req Service request
  * \param res Service response
  */
  bool srvCallback_SetOperationMode(cob_srvs::SetString::Request &req, cob_srvs::SetString::Response &res)
  {
    if (req.data != "velocity")
    {
      ROS_WARN("Powercube chain currently only supports velocity commands");
      res.success = false;
    }
    else
    {
      res.success = true;
    }
    return true;
  }

  /*!
   * \brief Publishes the state of the powercube_chain as ros messages.
   *
   * Published to "/joint_states" as "sensor_msgs/JointState"
   * Published to "state" as "control_msgs/JointTrajectoryControllerState"
   */
  void publishState(bool update = true)
  {
    if (initialized_)
    {
      ROS_DEBUG("publish state");

      if (update)
      {
        pc_ctrl_->updateStates();
      }

      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.header.stamp = ros::Time::now();
      joint_state_msg.name = pc_params_->GetJointNames();
      joint_state_msg.position = pc_ctrl_->getPositions();
      joint_state_msg.velocity = pc_ctrl_->getVelocities();
      joint_state_msg.effort.resize(pc_params_->GetDOF());

      control_msgs::JointTrajectoryControllerState controller_state_msg;
      controller_state_msg.header.stamp = joint_state_msg.header.stamp;
      controller_state_msg.joint_names = pc_params_->GetJointNames();
      controller_state_msg.actual.positions = pc_ctrl_->getPositions();
      controller_state_msg.actual.velocities = pc_ctrl_->getVelocities();
      controller_state_msg.actual.accelerations = pc_ctrl_->getAccelerations();

      std_msgs::String opmode_msg;
      opmode_msg.data = "velocity";

      /// publishing joint and controller states on topic
      topicPub_JointState_.publish(joint_state_msg);
      topicPub_ControllerState_.publish(controller_state_msg);
      topicPub_OperationMode_.publish(opmode_msg);

      last_publish_time_ = joint_state_msg.header.stamp;
    }

    // check status of PowerCube chain
    if (pc_ctrl_->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK)
    {
      error_ = true;
    }

    // check status of PowerCube chain
    if (pc_ctrl_->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK)
    {
      error_ = true;
    }
    else
    {
      error_ = false;
    }

    // publishing diagnotic messages
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.status.resize(1);

    // set data to diagnostics
    if (error_)
    {
      diagnostics.status[0].level = 2;
      diagnostics.status[0].name = n_.getNamespace();
      diagnostics.status[0].message = pc_ctrl_->getErrorMessage();
    }
    else
    {
      if (initialized_)
      {
        diagnostics.status[0].level = 0;
        diagnostics.status[0].name = n_.getNamespace();  //"schunk_powercube_chain";
        diagnostics.status[0].message = "powercubechain initialized and running";
      }
      else
      {
        diagnostics.status[0].level = 1;
        diagnostics.status[0].name = n_.getNamespace();  //"schunk_powercube_chain";
        diagnostics.status[0].message = "powercubechain not initialized";
      }
    }
    // publish diagnostic message
    topicPub_Diagnostic_.publish(diagnostics);
  }
};  // PowerCubeChainNode

/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc, char **argv)
{
  /// initialize ROS, specify name of node
  ros::init(argc, argv, "powercube_chain");

  // create PowerCubeChainNode
  PowerCubeChainNode pc_node;
  pc_node.getROSParameters();
  pc_node.getRobotDescriptionParameters();

  /// get main loop parameters
  double frequency;
  if (pc_node.n_private_.hasParam("frequency"))
  {
    pc_node.n_private_.getParam("frequency", frequency);
    // frequency of driver has to be much higher then controller frequency
  }
  else
  {
    // frequency of driver has to be much higher then controller frequency
    frequency = 10;  // Hz
    ROS_WARN("Parameter 'frequency' not available, setting to default value: %f Hz", frequency);
  }

  ros::Duration min_publish_duration;
  if (pc_node.n_private_.hasParam("min_publish_duration"))
  {
    double sec;
    pc_node.n_private_.getParam("min_publish_duration", sec);
    min_publish_duration.fromSec(sec);
  }
  else
  {
    ROS_ERROR("Parameter 'min_publish_time' not available");
    return 0;
  }

  if ((1.0 / min_publish_duration.toSec()) > frequency)
  {
    ROS_ERROR("min_publish_duration has to be longer then delta_t of controller frequency!");
    return 0;
  }

  /// main loop
  ros::Rate loop_rate(frequency);  // Hz
  while (pc_node.n_.ok())
  {
    if ((ros::Time::now() - pc_node.last_publish_time_) >= min_publish_duration)
    {
      pc_node.publishState();
    }

    /// sleep and waiting for messages, callbacks
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


