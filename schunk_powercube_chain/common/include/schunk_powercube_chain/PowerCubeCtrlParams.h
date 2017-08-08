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


#ifndef __POWER_CUBE_CTRL_PARAMS_H_
#define __POWER_CUBE_CTRL_PARAMS_H_

/*!
 * \brief Parameters for cob_powercube_chain
 *
 * Initializing and setting parameters for cob_powercube_chain
 */
class PowerCubeCtrlParams
{
public:
  /// Constructor
  PowerCubeCtrlParams()
  {
    m_DOF = 0;
    m_UseMoveVel = true;
  };

  /// Destructor
  ~PowerCubeCtrlParams();

  /// Initializing
  int Init(std::string CanModule, std::string CanDevice, int Baudrate, std::vector<int> ModuleIDs)
  {
    SetCanModule(CanModule);
    SetCanDevice(CanDevice);
    SetBaudrate(Baudrate);
    SetDOF(ModuleIDs.size());
    for (int i = 0; i < m_DOF; i++)
    {
      m_ModulIDs.push_back(ModuleIDs[i]);
    }
    return 0;
  }

  /// Sets the DOF value
  void SetDOF(int DOF)
  {
    m_DOF = DOF;
  }

  /// Gets the DOF value
  int GetDOF()
  {
    return m_DOF;
  }

  /// Sets UseMoveVel
  void SetUseMoveVel(bool UseMoveVel)
  {
    m_UseMoveVel = UseMoveVel;
  }

  /// Gets UseMoveVel
  int GetUseMoveVel()
  {
    return m_UseMoveVel;
  }

  /// Sets the CAN Module
  void SetCanModule(std::string CanModule)
  {
    m_CanModule = CanModule;
  }

  /// Gets the CAN Module
  std::string GetCanModule()
  {
    return m_CanModule;
  }

  /// Sets the CAN Device
  void SetCanDevice(std::string CanDevice)
  {
    m_CanDevice = CanDevice;
  }

  /// Gets the CAN Device
  std::string GetCanDevice()
  {
    return m_CanDevice;
  }

  /// Sets the Baudrate
  void SetBaudrate(int Baudrate)
  {
    m_Baudrate = Baudrate;
  }

  /// Gets the Baudrate
  int GetBaudrate()
  {
    return m_Baudrate;
  }

  /// Gets the Module IDs
  std::vector<int> GetModuleIDs()
  {
    return m_ModulIDs;
  }

  /// Gets the ModuleID
  int GetModuleID(int no)
  {
    if (no < GetDOF())
      return m_ModulIDs[no];
    else
      return -1;
  }

  /// Sets the Module IDs
  int SetModuleID(int no, int id)
  {
    if (no < GetDOF())
    {
      m_ModulIDs[no] = id;
      return 0;
    }
    else
      return -1;
  }

  /// Gets the joint names
  std::vector<std::string> GetJointNames()
  {
    return m_JointNames;
  }

  /// Sets the joint names
  int SetJointNames(std::vector<std::string> JointNames)
  {
    if ((int)JointNames.size() == GetDOF())
    {
      m_JointNames = JointNames;
      return 0;
    }
    else
      return -1;
  }
  // ToDo: Check the following

  ////////////////////////////////////////
  // Functions for angular constraints: //
  ////////////////////////////////////////

  /// Sets the upper angular limits (rad) for the joints
  int SetUpperLimits(std::vector<double> UpperLimits)
  {
    if ((int)UpperLimits.size() == GetDOF())
    {
      m_UpperLimits = UpperLimits;
      return 0;
    }
    return -1;
  }

  /// Sets the lower angular limits (rad) for the joints
  int SetLowerLimits(std::vector<double> LowerLimits)
  {
    if ((int)LowerLimits.size() == GetDOF())
    {
      m_LowerLimits = LowerLimits;
      return 0;
    }
    return -1;
  }

  /// Sets the offset angulars (rad) for the joints
  int SetOffsets(std::vector<double> AngleOffsets)
  {
    if ((int)AngleOffsets.size() == GetDOF())
    {
      m_Offsets = AngleOffsets;
      return 0;
    }
    return -1;
  }

  /// Sets the max. angular velocities (rad/s) for the joints
  int SetMaxVel(std::vector<double> MaxVel)
  {
    if ((int)MaxVel.size() == GetDOF())
    {
      m_MaxVel = MaxVel;
      return 0;
    }
    return -1;
  }

  /// Sets the max. angular accelerations (rad/s^2) for the joints
  int SetMaxAcc(std::vector<double> MaxAcc)
  {
    if ((int)MaxAcc.size() == GetDOF())
    {
      m_MaxAcc = MaxAcc;
      return 0;
    }
    return -1;
  }

  /// Gets the upper angular limits (rad) for the joints
  std::vector<double> GetUpperLimits()
  {
    return m_UpperLimits;
  }

  /// Gets the lower angular limits (rad) for the joints
  std::vector<double> GetLowerLimits()
  {
    return m_LowerLimits;
  }

  /// Gets the offset angulars (rad) for the joints
  std::vector<double> GetOffsets()
  {
    return m_Offsets;
  }

  /// Gets the max. angular accelerations (rad/s^2) for the joints
  std::vector<double> GetMaxAcc()
  {
    return m_MaxAcc;
  }

  /// Gets the max. angular velocities (rad/s) for the joints
  std::vector<double> GetMaxVel()
  {
    return m_MaxVel;
  }

private:
  int m_DOF;
  std::vector<int> m_ModulIDs;
  std::vector<std::string> m_JointNames;
  std::string m_CanModule;
  std::string m_CanDevice;
  int m_Baudrate;
  bool m_UseMoveVel;
  std::vector<double> m_Offsets;
  std::vector<double> m_UpperLimits;
  std::vector<double> m_LowerLimits;
  std::vector<double> m_MaxVel;
  std::vector<double> m_MaxAcc;
};

#endif
