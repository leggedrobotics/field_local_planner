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
/**
 *  @file  MotionPolicy.hpp
 *  @brief Implements motion policies (accelerations)
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <iostream>

namespace gtsam {
namespace rmp {

class MotionPolicy {
 public:
  static Vector3 makeGradientPosePolicy(Vector2 gradient, double gain) {
    constexpr float eps = 1e-6;
    double grad_norm = gradient.norm();
    Vector2 unit_gradient = grad_norm < eps? gradient : gradient / grad_norm;

    Rot2 heading = Rot2::atan2(unit_gradient.y(), unit_gradient.x());
    return gain * Vector3(unit_gradient.x(), unit_gradient.y(), heading.theta());
  }

  static Vector3 makeGradientPositionPolicy(Vector2 gradient, double gain) {
    Vector3 acc = makeGradientPosePolicy(gradient, gain);
    acc(2) = 0.0;  // Remove rotational components
    return acc;
  }

  static Vector3 makeGradientOrientationPolicy(Vector2 gradient, double gain) {
    Vector3 acc = makeGradientPosePolicy(gradient, gain);
    acc(0) = 0.0;  // Remove translational components
    acc(1) = 0.0;  // Remove translational components
    return acc;
  }

  static Vector3 makeGoalPosePolicy(const Pose2& state, const Pose2& goal, double gain) {
    // Compute difference to goal
    Vector3 acc = traits<Pose2>::Local(state, goal);
    return gain * acc;
  }

  static Vector3 makeGoalPositionPolicy(const Pose2& state, const Pose2& goal, double gain) {
    Vector3 acc = makeGoalPosePolicy(state, goal, gain);
    acc(2) = 0.0;  // Remove rotational components
    return acc;
  }

  static Vector3 makeGoalOrientationPolicy(const Pose2& state, const Pose2& goal, double gain) {
    Vector3 acc = makeGoalPosePolicy(state, goal, gain);
    acc(0) = 0.0;  // Remove translational components
    acc(1) = 0.0;  // Remove translational components
    return acc;
  }

  static Vector3 makeVelocityOrientationPolicy(const Vector3& velocity, const Vector2& direction, double gain) {
    Vector3 acc = Vector3::Zero();

    const double eps = 1e-3;

    // Get direction of velocity
    Rot2 dir_velocity = Rot2::atan2(velocity(1), velocity(0));
    // Get direction of robot
    Rot2 dir_forward = Rot2::atan2(direction(1), direction(0));
    // Get direction
    double vel_dir_norm = velocity.head<2>().norm();
    // Compute angle of correction
    Rot2 angle_forward = dir_forward.inverse() * dir_velocity;

    // Fill output
    // Compute output acceleration
    double theta_forward = angle_forward.theta();

    double theta = theta_forward;
    theta = (theta > M_PI_2) ? theta - M_PI : theta;
    theta = (theta < -M_PI_2) ? theta + M_PI : theta;

    if (vel_dir_norm > eps) {
      acc(2) = gain * theta;
    }

    return acc;
  }

  static Vector3 makeDampingPolicy(const Vector3& velocity, double gain) {
    //
    return -gain * velocity;
  }

  static Vector2 makeObstaclePolicy(const Vector2& gradient, const Vector2& velocity, double distance, double gain) {
    double d = std::max(distance, 0.01);
    constexpr float eps = 1e-6;
    double grad_norm = gradient.norm();
    Vector2 unit_gradient = grad_norm < eps? gradient : gradient / grad_norm;
    double velocity_alignment_gain = unit_gradient.dot(velocity);
    
    return -(gain / d) * velocity_alignment_gain * unit_gradient;
  }

  static Vector2 makeObstacleDampingPolicy(const Vector2& gradient, const Vector2& velocity, double distance, double gain) {
    double d = std::max(distance, 0.01);
    constexpr float eps = 1e-6;
    double grad_norm = gradient.norm();
    Vector2 unit_gradient = grad_norm < eps? gradient : gradient / grad_norm;

    return -(gain / d) * unit_gradient;
  }
};

}  // namespace rmp
}  // namespace gtsam