//----------------------------------------
// This file is part of field_local_planner
//
// Copyright (C) 2020-2025 Matías Mattamala, University of Oxford.
//
// field_local_planner is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// field_local_planner is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with field_local_planner.
// If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------
#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

#include <field_local_planner_base/basic_types.hpp>

namespace field_local_planner {
namespace utils {

#define COL_WIDTH 40

using namespace gtsam;
//-------------------------------------------------------------------------------------------------
// Data converters
//-------------------------------------------------------------------------------------------------
static inline Pose2 toSE2(const Pose3& T_SE3) {
  return Pose2(T_SE3.translation().x(), T_SE3.translation().y(), T_SE3.rotation().yaw());
}

static inline Time fromSeconds(const double& t) {
  return (int64_t)floor(t * 1e6);
}

//-------------------------------------------------------------------------------------------------
// Math utils
//-------------------------------------------------------------------------------------------------
static inline double clipValue(double value, double max_value) {
  return std::max(std::min(value, max_value), -max_value);
}

// constrain angle to be -180:180 in radians
static inline double constrainAngle(double x) {
  x = fmod(x + M_PI, 2.0 * M_PI);
  if (x < 0) x += 2.0 * M_PI;
  return x - M_PI;
}

/**
 * Convert the angle given in radians to degrees.
 * https://techoverflow.net/2019/05/30/c-equivalent-of-numpy-php-rad2deg/
 */
template <typename T>
static T rad2deg(T angle) {
  return angle * 180.0 / M_PI;
}
template <typename T>
static T deg2rad(T angle) {
  return angle / 180.0 * M_PI;
}

// Bessel function 1st kind, order 0
// from https://www.atnf.csiro.au/computing/software/gipsy/sub/bessel.c
static double bessj0(double x) {
  double ax, z;
  double xx, y, ans, ans1, ans2;

  if ((ax = fabs(x)) < 8.0) {
    y = x * x;
    ans1 = 57568490574.0 + y * (-13362590354.0 + y * (651619640.7 + y * (-11214424.18 + y * (77392.33017 + y * (-184.9052456)))));
    ans2 = 57568490411.0 + y * (1029532985.0 + y * (9494680.718 + y * (59272.64853 + y * (267.8532712 + y * 1.0))));
    ans = ans1 / ans2;
  } else {
    z = 8.0 / ax;
    y = z * z;
    xx = ax - 0.785398164;
    ans1 = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 + y * (-0.2073370639e-5 + y * 0.2093887211e-6)));
    ans2 = -0.1562499995e-1 + y * (0.1430488765e-3 + y * (-0.6911147651e-5 + y * (0.7621095161e-6 - y * 0.934935152e-7)));
    ans = sqrt(0.636619772 / ax) * (cos(xx) * ans1 - z * sin(xx) * ans2);
  }
  return ans;
}

// Von mises distribution
static double vonMises(double x, double mu, double kappa) {
  return exp(kappa * cos(x - mu)) /*/ (2 * M_PI * bessj0(kappa)) */;
}

// Rotation utils
inline double mod2pi_positive(double vin) {
  double q = vin / (2 * M_PI) + 0.5;
  int qi = (int)q;

  return vin - qi * 2 * M_PI;
}

/** Map v to [-PI, PI] **/
inline double mod2pi(double vin) {
  if (vin < 0)
    return -mod2pi_positive(-vin);
  else
    return mod2pi_positive(vin);
}

}  // namespace utils
}  // namespace field_local_planner