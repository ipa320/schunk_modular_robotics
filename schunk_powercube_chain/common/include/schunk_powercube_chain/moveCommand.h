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


#ifndef __MOVE_COMMAND_H__
#define __MOVE_COMMAND_H__

#include <schunk_powercube_chain/datastructsManipulator.h>
#include <schunk_powercube_chain/TimeStamp.h>

class moveCommand
{
public:
  moveCommand() {;}
  virtual ~moveCommand() {;}

  /////////////////////////////////////////////////////////////////////////
  //               Child Classes have to define these:                   //
  /////////////////////////////////////////////////////////////////////////

  /*!
   * \brief returns the planned position for TimeElapsed (seconds)
   */
  virtual double getPos(double TimeElapsed) = 0;

  /*!
   * \brief returns the planned velocity for TimeElapsed (seconds)
   */
  virtual double getVel(double TimeElapsed) = 0;

  /*!
   * \brief returns the planned total time for the movement (in seconds)
   */
  virtual double getTotalTime() = 0;

  /////////////////////////////////////////////////////////////////////////
  //                Functions useful for simulation:                     //
  //          start() has to be called before the other functions        //
  /////////////////////////////////////////////////////////////////////////

  /*!
   * \brief sets the starttime for the movement to the current time
   */
  virtual void start()
  {
    m_timeStarted.SetNow();
  }

  /*!
   * \brief returns true if the the end of the movement is not reached yet
   */
  virtual bool isActive()
  {
    m_now.SetNow();
    return (m_now - m_timeStarted > getTotalTime()) ? false : true;
    std::cerr << "========Total Time: " << getTotalTime() << "\n";
  }

  /*!
   * \brief returns the planned position at time of function call (desired position)
   */
  virtual double pos()
  {
    m_now.SetNow();
    return getPos(m_now - m_timeStarted);
  }

  /*!
   * \brief returns the planned velocity at time of function call (desired velocitys)
   */
  virtual double vel()
  {
    m_now.SetNow();
    return getVel(m_now - m_timeStarted);
  }

  /*!
   * \brief returns remaining time
   */
  virtual double timeRemaining()
  {
    m_now.SetNow();
    return (m_timeStarted - m_now) + getTotalTime();
  }

protected:
  TimeStamp m_timeStarted;
  TimeStamp m_now;
};

class RampCommand : public moveCommand
{
public:
  RampCommand(double x0, double v0, double xtarget, double amax, double vmax);
  RampCommand(const RampCommand& rc);

  virtual RampCommand& operator=(const RampCommand& rc);

  virtual ~RampCommand()
  {
    if (m_nachumkehr)
      delete m_nachumkehr;
  }

  /*!
   * \brief returns the planned position for TimeElapsed (seconds)
   */
  virtual double getPos(double TimeElapsed);
  double getPos()
  {
    return moveCommand::pos();
  }

  /*!
   * \brief returns the planned velocity for TimeElapsed (seconds)
   */
  virtual double getVel(double TimeElapsed);
  double getVel()
  {
    return moveCommand::vel();
  }

  /*!
   * \brief returns the planned total time for the movement (in seconds)
   */
  virtual double getTotalTime();

  virtual bool inPhase1()
  {
    m_now.SetNow();
    return (m_now - m_timeStarted <= m_T1) ? true : false;
  }
  virtual bool inPhase3()
  {
    m_now.SetNow();
    return (m_now - m_timeStarted > m_T1 + m_T2 && m_now - m_timeStarted < m_T1 + m_T2 + m_T3) ? true : false;
  }

  /*!
   * \brief Return the times of the different phases of the ramp move
   */
  virtual double T1()
  {
    return (m_umkehr) ? (m_T1 + m_nachumkehr->T1()) : m_T1;
  }
  virtual double T2()
  {
    return (m_umkehr) ? m_nachumkehr->T2() : m_T2;
  }
  virtual double T3()
  {
    return (m_umkehr) ? m_nachumkehr->T3() : m_T3;
  }

  /*!
   * \brief Calculate the necessary a and v of a rampmove, so that the move will take the desired time
   */
  static void calculateAV(double x0, double v0, double xtarget, double time, double T3, double amax, double vmax,
                          double& a, double& v);

private:
  static std::ofstream debug;

  double m_x0, m_v0;
  double m_xtarget;
  double m_amax, m_vmax;

  double m_T1, m_T2, m_T3;
  double m_a1, m_v2, m_a3;
  bool m_umkehr;
  RampCommand* m_nachumkehr;
};

#endif
