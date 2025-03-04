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
#include <math.h>
#include <field_local_planner_trackline/trackline.hpp>

namespace field_local_planner {

Trackline::Trackline() : BaseLocalPlanner(), state_(State::UNKNOWN) {
}

void Trackline::setParameters(const Trackline::Parameters& p) {
  parameters_ = p;
}

Trackline::Parameters Trackline::getParameters() const {
  return parameters_;
}

Twist Trackline::computeTwist() {
  // Check states
  if (BaseLocalPlanner::distance_to_goal_ > base_parameters_.distance_to_goal_thr) {
    // If the robot is far from the goal, check heading towards goal
    if (std::fabs(BaseLocalPlanner::heading_towards_goal_) > base_parameters_.orientation_to_goal_thr) {
      // If it's not pointing towards the goal, correct heading
      // std::cout << "Trackline: TURN_TO_GOAL" << std::endl;
      stateTurnToGoal();

    } else {
      // If it's pointing towards the goal, move forward tracking the line
      // std::cout << "Trackline: FORWARD" << std::endl;
      stateForward();

    }
  } else {
    // std::cout << "Trackline: TURN_TO_DESTINATION" << std::endl;
    // If it's close to the goal, match the final orientation
    stateTurnToDestination();
  }

  // Compute final velocity
  double output_linvel_x = goal_linvel_x_ + trackline_linvel_x_;
  double output_linvel_y = goal_linvel_y_ + trackline_linvel_y_;

  double max_forward_linear_velocity_now = base_parameters_.max_linear_velocity_x;
  double max_lateral_linear_velocity_now = base_parameters_.max_linear_velocity_y;

  double ratio_x = fabs(output_linvel_x) / max_forward_linear_velocity_now;
  if (ratio_x < 1) ratio_x = 1;

  output_linvel_x = output_linvel_x / ratio_x;
  output_linvel_y = output_linvel_y / ratio_x;

  double ratio_y = fabs(output_linvel_y) / max_lateral_linear_velocity_now;
  if (ratio_y < 1) ratio_y = 1;
  output_linvel_y = output_linvel_y / ratio_y;
  output_linvel_x = output_linvel_x / ratio_y;

  // if we are considering the robot's front as the back, need to invert the
  // control commands on the x and y axes for it to move correctly.
  Twist twist;
  twist(0) = 0.0;              // Angular x
  twist(1) = 0.0;              // Angular y
  twist(2) = goal_angvel_z_;   // Angular z
  twist(3) = output_linvel_x;  // Linear x
  twist(4) = base_parameters_.differential_mode? 0.0 : output_linvel_y;  // Linear y
  twist(5) = 0.0;              // Linear z

  return twist;
}

Path Trackline::computePath() {
  Path path;
  
  // Compute pose of starting pose in robot frame
  Pose3 T_b_start = T_f_b_.inverse() * T_f_b_start_;
  
  // And pose of goal in current frame
  Pose3 T_b_goal = T_f_b_.inverse() * T_f_g_;
  
  path.push_back(T_b_start);
  path.push_back(T_b_goal);

  return path;
}

void Trackline::stateTurnToGoal() {
  goal_angvel_z_ = utils::clipValue(heading_towards_goal_ * parameters_.angular_gain_p, base_parameters_.max_angular_velocity_z);
  goal_linvel_x_ = 0.0; //std::cos(orientation_to_start_) * base_parameters_.max_linear_velocity_x;  // m/s
  goal_linvel_y_ = 0.0; //std::sin(orientation_to_start_) * parameters_.max_linear_velocity_y;  // m/s
  trackline_linvel_x_ = 0;
  trackline_linvel_y_ = 0;
}

void Trackline::stateForward() {
  goal_angvel_z_ = utils::clipValue(heading_towards_goal_ * parameters_.angular_gain_p, base_parameters_.max_angular_velocity_z);
  goal_linvel_x_ = cos(heading_towards_goal_) * base_parameters_.max_linear_velocity_x;  // m/s
  goal_linvel_y_ = sin(heading_towards_goal_) * base_parameters_.max_linear_velocity_x;  // m/s

  // How far is the robot from the trackline?
  // Create an x/y base velocity term proportional to that distance

  // signed perpendicular distance between pt0 and line (between pt1 and pt2)
  // point: 0  current_pose
  // line: 1  starting_pose and  2  current_goal
  // distance is negative on LHS of line. positive on RHS of line
  double x0, y0, x1, y1, x2, y2;
  x0 = T_f_b_.translation().x();
  y0 = T_f_b_.translation().y();
  x1 = T_f_b_start_.translation().x();
  y1 = T_f_b_start_.translation().y();
  x2 = T_f_g_.translation().x();
  y2 = T_f_g_.translation().y();
  double numer = (x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1);
  double denom = std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
  double trackline_distance_error = numer / denom;
  double slope = std::atan2((y2 - y1), (x2 - x1));

  double current_yaw = dT_b_start_.rotation().yaw();

  // This is the angle between the trackline and the direction the robot is facing
  // this is zero if the robot is on trackline and facing goal
  // it is negative on LHS of line. positive on RHS of line
  double projection_angle = current_yaw - slope;

  // Create velocity command proportional to error
  double trackline_velocity = trackline_distance_error * parameters_.linear_gain_p;
  if (trackline_velocity > base_parameters_.max_linear_velocity_x) trackline_velocity = base_parameters_.max_linear_velocity_x;
  if (trackline_velocity < -base_parameters_.max_linear_velocity_x) trackline_velocity = -base_parameters_.max_linear_velocity_x;

  trackline_linvel_x_ = trackline_velocity * sin(projection_angle);
  trackline_linvel_y_ = trackline_velocity * cos(projection_angle);
}

void Trackline::stateTurnToDestination() {
  goal_angvel_z_ = utils::clipValue(orientation_to_goal_ * parameters_.angular_gain_p, base_parameters_.max_angular_velocity_z);

  goal_linvel_x_ = 0.0; //cos(heading_towards_goal_) * base_parameters_.max_linear_velocity_x;  // m/s
  goal_linvel_y_ = 0.0; //sin(heading_towards_goal_) * base_parameters_.max_linear_velocity_x;  // m/s

  trackline_linvel_x_ = 0;
  trackline_linvel_y_ = 0;
}

}  // namespace field_local_planner