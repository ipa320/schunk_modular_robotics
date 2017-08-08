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


#ifndef _SIMULATED_MOTOR_H_
#define _SIMULATED_MOTOR_H_

#include <schunk_powercube_chain/moveCommand.h>
#include <string>

class simulatedMotor
{
public:
  simulatedMotor(double lowLimit, double upLimit, double maxAcc, double maxVel);
  ~simulatedMotor() {;}

  /// @brief initializes the module, if an error occurs function returns false
  virtual bool init()
  {
    return true;
  }

  /// @brief if error occured during init, get the error message with this
  virtual std::string getErrorMessage()
  {
    return std::string("No Errors.");
  }

  /// @brief any messages useful for debug are sent to this stream:
  // virtual void setDebugOutput(ostream* os) { deb = os; }

  /////////////////////////////////////////
  // Zunächst die Steuerungs-Funktionen: //
  /////////////////////////////////////////

  /// @brief executes a rampmove
  virtual void moveRamp(double targetAngle, double vmax, double amax);

  /// @brief Moves the motor with the given velocity
  virtual void moveVel(double vel);

  /// @brief Moves the motor with the given velocity
  virtual void movePos(double pos);

  /// @brief Stops the motor immediately
  virtual void stop();

  ////////////////////////////////////////////////////////
  // Funktionen zum setzen und auslesen von Parametern: //
  ////////////////////////////////////////////////////////

  /// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
  /// A Value of 0.5 is already pretty fast, you probably don't want anything more than 1.0...
  virtual void setMaxVelocity(double radpersec)
  {
    m_vmax = radpersec;
  }
  virtual double getMaxVelocity()
  {
    return m_vmax;
  }

  /// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
  /// A Value of 0.5 is already pretty fast, you probably don't want anything more than 1.0...
  virtual void setMaxAcceleration(double radPerSecSquared)
  {
    m_amax = radPerSecSquared;
  }
  virtual double getMaxAcceleration()
  {
    return m_amax;
  }

  /// @brief sets the Joint Limits, the motor has to stay in these limits!
  virtual void setLimits(double lowerLimit, double upperLimit)
  {
    m_ul = upperLimit;
    m_ll = lowerLimit;
  }
  virtual double getUpperLimit()
  {
    return m_ul;
  }
  virtual double getLowerLimit()
  {
    return m_ll;
  }

  /// @brief sets / gets the time that the motor needs to reach target velocity (moveVel)
  virtual void setTimeConstant(double T)
  {
    T0 = T;
  }
  virtual double getTimeConstant() const
  {
    return T0;
  }

  ////////////////////////////////////////////
  // hier die Funktionen zur Statusabfrage: //
  ////////////////////////////////////////////

  /// @brief Get a representation of the rampMove that the motor WOULD execute if told so at the time of function call
  virtual RampCommand getRampMove(double targetAngle, double v, double a);

  /// @brief Same but using the maximum velocity and acceleration of this motor
  virtual RampCommand getRampMove(double targetAngle)
  {
    return getRampMove(targetAngle, m_vmax, m_amax);
  }

  /// @brief Returns the current Joint Angles
  virtual double getAngle()
  {
    return m_lastMove.getPos();
  }

  /// @brief Returns the current Angular velocities (Rad/s)
  virtual double getVelocity()
  {
    return m_lastMove.getVel();
  }

  /// @brief Returns true if the Joint is still moving
  /// also returns true if Joints are accelerating or decelerating
  virtual bool statusMoving()
  {
    return m_lastMove.isActive();
  }

  /// @brief Returns true if the Joint is decelerating at end of movement
  virtual bool statusDec()
  {
    return m_lastMove.inPhase3();
  }

  /// @brief Returs true if the Joint is in phase one of rampmove
  virtual bool statusAcc()
  {
    return m_lastMove.inPhase1();
  }

private:
  RampCommand m_lastMove;

  double m_ul, m_ll;      // upper limit and lower limit
  double m_amax, m_vmax;  // Never mover faster than this!

  double T0;  // Zeitkonstante für Annäherung der Sprungantwort durch Rampe

  // ostream * deb;
};

#endif
