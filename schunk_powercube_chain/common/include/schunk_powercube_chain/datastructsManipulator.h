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


#ifndef __DATASTRUCTS_MANIPULATOR_H_
#define __DATASTRUCTS_MANIPULATOR_H_

#include <schunk_powercube_chain/Joint.h>
#include <iostream>

// never in headerfiles!
// using namespace std;

#ifdef SWIG
%module PowerCubeCtrl
%{
  #include <schunk_powercube_chain/datastructsManipulator.h>
%}
#endif

/**
 * @brief Definition of the used structs in the project Path planner
 * @author Katrin Thurow <katrinthurow@hotmail.com>
 */

struct Point3D
{
  double x;
  double y;
  double z;
};

struct AbsPos
{
  AbsPos() : Eulerx(0.0), Eulery(0.0), Eulerz(0.0), Transx(0.0), Transy(0.0), Transz(0.0)
  {
    ;
  }
  // Bitte beachten: Es werden von nun an XYZ-fixed Winkel benutzt
  // (Siehe Craig, S.45ff)
  // welche gleichbedeutend mit den Euler-ZYX Winkeln sind.
  // Die Bezeichnungen Eulerx, Eulery, Eulerz werden daher beibehalten.
  double Eulerx;
  double Eulery;
  double Eulerz;
  double Transx;
  double Transy;
  double Transz;
  void set(double* p);
  void setTransX(double transx)
  {
    Transx = transx;
  }
  void setTransY(double transy)
  {
    Transy = transy;
  }
  void setTransZ(double transz)
  {
    Transz = transz;
  }
  void setEulerX(double eulerx)
  {
    Eulerx = eulerx;
  }
  void setEulerY(double eulery)
  {
    Eulery = eulery;
  }
  void setEulerZ(double eulerz)
  {
    Eulerz = eulerz;
  }
  double getTransX()
  {
    return Transx;
  }
  double getTransY()
  {
    return Transy;
  }
  double getTransZ()
  {
    return Transz;
  }
  double getEulerX()
  {
    return Eulerx;
  }
  double getEulerY()
  {
    return Eulery;
  }
  double getEulerZ()
  {
    return Eulerz;
  }
  void angleScale(double s);
  void toDeg()
  {
    angleScale(57.295779524);
  }
  void toRad()
  {
    angleScale(0.017453292);
  }
  AbsPos operator*(double s) const;
  AbsPos operator+(const AbsPos& abs2) const;
  AbsPos operator-(const AbsPos& abs2) const;
  double getPosLength() const
  {
    return sqrt(Transx * Transx + Transy * Transy + Transz * Transz);
  }
  /// @brief in distMeasure(otherPos) wird die Winkelabweichung berücksichtigt
  /// Hierbei wird der RMS der Differenzen der Eulerwinkel benutzt
  /// Wenn Trans... in mm wird 1° Abweichung wie 1mm Abweichung gewichtet.
  /// TODO: Wünschenswert wäre ein besseres Maß für die Lagedifferenz (z.B. Angle-Axis-Wikel)
  double distMeasure(AbsPos otherPos) const;
};

std::ostream& operator<<(std::ostream& os, const AbsPos& a);

inline AbsPos operator*(double s, const AbsPos& abs)
{
  return abs * s;
}

struct DH
{
  Jointd a;
  Jointd d;
  Jointd alpha;
  Jointd theta;
};

struct LimitsTheta
{
  Jointd max;
  Jointd min;
};

#endif  //__DATASTRUCTS_MANIPULATOR_H_
