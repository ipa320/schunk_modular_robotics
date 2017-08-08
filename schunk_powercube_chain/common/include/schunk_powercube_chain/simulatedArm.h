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


#ifndef _SIMULATED_ARM_H_
#define _SIMULATED_ARM_H_

#include <schunk_powercube_chain/PowerCubeCtrl.h>
#include <vector>
#include <string>

class simulatedMotor;

class simulatedArm
{
public:
  simulatedArm();
  virtual ~simulatedArm();

  bool Init(PowerCubeCtrlParams* params);

  bool isInitialized() const
  {
    return m_Initialized;
  }

  std::string getErrorMessage() const
  {
    return m_ErrorMessage;
  }

  bool Close()
  {
    m_Initialized = false;
    return true;
  }

  /////////////////////////////////
  // Funktionen Arm-Ansteuerung: //
  /////////////////////////////////

  /// @brief same as MoveJointSpace, but final angles should by reached simultaniously!
  /// Returns the time that the movement will take
  bool MoveJointSpaceSync(const std::vector<double>& Angle);

  /// @brief moves all cubes to the given position
  bool MovePos(const std::vector<double>&);
  /// @brief Moves all cubes by the given velocities
  bool MoveVel(const std::vector<double>&);

  /// @brief current movement currently not supported in simulation
  //  bool MoveCur(const std::vector<double>&);

  /// @brief Stops the Manipulator immediately
  bool Stop();

  ///////////////////////////////////////////
  // Funktionen zum setzen von Parametern: //
  ///////////////////////////////////////////

  /// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
  /// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
  bool setMaxVelocity(double radpersec);
  bool setMaxVelocity(const std::vector<double>& radpersec);

  /// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
  /// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
  bool setMaxAcceleration(double radPerSecSquared);
  bool setMaxAcceleration(const std::vector<double>& radPerSecSquared);

  ////////////////////////////////////////////
  // hier die Funktionen zur Statusabfrage: //
  ////////////////////////////////////////////

  /// @brief Returns the current Joint Angles
  bool getConfig(std::vector<double>& result);

  /// @brief Returns the current Angular velocities (Rad/s)
  bool getJointVelocities(std::vector<double>& result);

  /// @brief Returns true if any of the Joints are still moving
  /// Should also return true if Joints are accelerating or decelerating
  bool statusMoving();

  /// @brief Returns true if any of the Joints are decelerating
  bool statusDec();

  /// @brief Returs true if any of the Joints are accelerating
  bool statusAcc();

  /// @brief Waits until all Modules are homed, writes status comments to out.
  // bool doHoming();
  // bool HomingDone();

  /// @brief Tells the Modules not to start moving until PCubel_startMotionAll is called
  bool waitForSync()
  {
    return true;
  }  // makes no difference in simulation
  /// @brief Execute move commands immediately from now on:
  bool dontWaitForSync()
  {
    return true;
  }  // makes no difference in simulation

protected:
  int m_DOF;
  bool m_Initialized;
  std::string m_ErrorMessage;

  std::vector<simulatedMotor> m_motors;

  std::vector<double> m_maxVel;
  std::vector<double> m_maxAcc;
};

#endif
