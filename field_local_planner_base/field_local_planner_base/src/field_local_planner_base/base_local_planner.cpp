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
#include <field_local_planner_base/base_local_planner.hpp>

namespace field_local_planner {

BaseLocalPlanner::BaseLocalPlanner() : state_(State::FINISHED), sensing_ready_(false), point_cloud_(new pcl::PointCloud<PointType>()) {}

void BaseLocalPlanner::setBaseParameters(const BaseLocalPlanner::Parameters& parameters) {
  base_parameters_ = parameters;
}

BaseLocalPlanner::Parameters BaseLocalPlanner::getBaseParameters() const {
  return base_parameters_;
}

bool BaseLocalPlanner::execute(const Time& ts, BaseLocalPlanner::Output& output) {
  if ((ts - last_ts_) < utils::fromSeconds(1 / base_parameters_.control_rate)) {
    output.status.state = state_;
    output.status.progress = progress_;
    output.status.progress_delta = progress_delta_;
    output.status.distance_to_goal = distance_to_goal_;
    output.status.orientation_to_goal = orientation_to_goal_;
    return false;
  }

  // Save current timestamp
  last_ts_ = ts;

  // Check distance and orientation to goal
  computeDistanceAndOrientationToGoal();

  // Check status of the controller
  state_ = checkState();

  // Fill state
  output.status.state = state_;
  output.status.progress = progress_;
  output.status.progress_delta = progress_delta_;
  output.status.distance_to_goal = distance_to_goal_;
  output.status.orientation_to_goal = orientation_to_goal_;

  Twist twist = Twist::Zero();
  Path path;

  if (state_ == State::EXECUTING) {
    // if ((ts - last_ts_) < utils::fromSeconds(1 / base_parameters_.control_rate)) {
    //   return false;
    // }

    // // Save current timestamp
    // last_ts_ = ts;

    // Compute twist - planner specific
    twist = computeTwist();
    twist = limitTwist(twist);

    // Compute path
    path = computePath();
  }

  // Make output
  output.twist = twist;
  output.path = path;

  return state_ == State::EXECUTING;
}

void BaseLocalPlanner::stop() {
  state_ = State::FINISHED;
}

// Other steps
void BaseLocalPlanner::computeDistanceAndOrientationToGoal() {
  // Note: All these are computed assuming planar SE(2) motion

  // Goal to current pose
  dT_b_g_ = T_f_b_.inverse() * T_f_g_;

  // Distance
  distance_to_goal_ = std::hypot(dT_b_g_.translation().y(), dT_b_g_.translation().x());
  // printf("BaseLocalPlanner: distance_to_goal_: %f\n", distance_to_goal_);

  // Orientation to goal
  Vector3 rpy_goal = dT_b_g_.rotation().rpy();
  orientation_to_goal_ = rpy_goal.z();
  // printf("BaseLocalPlanner: orientation_to_goal_: %f \n", orientation_to_goal_);

  // Heading to point towards goal
  heading_towards_goal_ = std::atan2(dT_b_g_.translation().y(), dT_b_g_.translation().x());
  // printf("BaseLocalPlanner: heading_towards_goal_: %f \n", heading_towards_goal_);

  // Starting pose to current
  dT_b_start_ = T_f_b_.inverse() * T_f_b_start_;

  // Distance
  distance_to_start_ = std::hypot(dT_b_start_.translation().y(), dT_b_start_.translation().x());
  // printf("BaseLocalPlanner: distance_to_start_: %f \n", distance_to_start_);

  // Orientation to start pose
  Vector3 rpy_start = dT_b_start_.rotation().rpy();
  orientation_to_start_ = rpy_start.z();
  // printf("BaseLocalPlanner: orientation_to_start_: %f \n", orientation_to_start_);
}

BaseLocalPlanner::State BaseLocalPlanner::checkState() {
  if (base_parameters_.requires_sensing && !sensing_ready_) {
    // std::cout << "BaseLocalPlanner: NOT_READY" << std::endl;
    return State::NOT_READY;
  }

  // Compute progress
  progress_ = 100.0 * (1.0 - distance_to_goal_ / distance_start_to_goal_);

  // printf("BaseLocalPlanner: distance_to_goal_ (%f) < base_parameters_.distance_to_goal_thr (%f)\n", distance_to_goal_,
  //  base_parameters_.distance_to_goal_thr);
  // printf("BaseLocalPlanner: orientation_to_goal_ (%f) < base_parameters_.orientation_to_goal_thr (%f)\n", orientation_to_goal_,
  //  base_parameters_.orientation_to_goal_thr);

  if (distance_to_goal_ < base_parameters_.distance_to_goal_thr                      // If the robot is close enough
      && std::fabs(orientation_to_goal_) < base_parameters_.orientation_to_goal_thr  // And the orientation is also close
  ) {
    // If the robot is close enough to the goal, then it must have finished
    return State::FINISHED;

  } else if (state_ != State::FINISHED && checkFailure()) {
    // If the robot doesn't make enough process, then we report failure
    return State::FAILURE;

  } else {
    // All good, it must be working
    return state_;
  }
}

bool BaseLocalPlanner::checkFailure() {
  // current time
  Time ts = ts_T_f_b_;

  // Compute delta progress
  progress_delta_ = (progress_ - last_progress_);
  // std::cout << "progress_delta_: " << progress_delta_ << std::endl;

  // Update last progress
  last_progress_ = progress_;
  ts_last_progress_ = ts;

  // Check if progress is close to zero
  if (std::fabs(progress_delta_) < base_parameters_.progress_threshold) {
    // std::cout << "Failure timeout: : " << (ts - ts_failure_) << " / " << utils::fromSeconds(base_parameters_.failure_timeout_sec)
    // << std::endl;
    if ((ts - ts_failure_) > utils::fromSeconds(base_parameters_.failure_timeout_sec)) {
      return true;  // Report failure
    }

  } else {
    ts_failure_ = ts_T_f_b_;  // Reset timer
  }

  // if (potential_failure_ && (ts - ts_failure_) > utils::fromSeconds(base_parameters_.failure_timeout_sec)) {
  //   return true;  // Report failure
  // }

  return false;
}

Twist BaseLocalPlanner::limitTwist(const Twist& twist) {
  Twist limited = twist;

  // Limit angular Z
  limited(2) = utils::clipValue(twist(2), base_parameters_.max_angular_velocity_z);
  // Limit linear X
  limited(3) = utils::clipValue(twist(3), base_parameters_.max_linear_velocity_x);
  // Limit linear Y
  limited(4) = utils::clipValue(twist(4), base_parameters_.max_linear_velocity_y);

  return limited;
}

// Interfaces for external data
void BaseLocalPlanner::setImageRgb(const cv::Mat& img, const Pose3& T_f_s, const Time& ts) {
  image_rgb_ = img.clone();
  T_f_s_rgb_ = T_f_s;
  ts_rgb_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setImageRgbd(const cv::Mat& img, const Pose3& T_f_s, const Time& ts) {
  image_rgbd_ = img.clone();
  T_f_s_rgbd_ = T_f_s;
  ts_rgbd_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setImageDepth(const cv::Mat& img, const Pose3& T_f_s, const Time& ts) {
  image_depth_ = img.clone();
  T_f_s_depth_ = T_f_s;
  ts_depth_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, const Pose3& T_f_s, const Time& ts) {
  point_cloud_ = cloud;
  T_f_s_pc_ = T_f_s;
  ts_pc_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setGridMap(const grid_map::GridMap& grid_map, const Pose3& T_f_s, const Time& ts) {
  grid_map_ = grid_map;
  T_f_s_gm_ = T_f_s;
  ts_gm_ = ts;
  sensing_ready_ = true;
}

// Set single goal
void BaseLocalPlanner::setGoalInFixed(const Pose3& T_f_g, const Pose3& T_f_b, const Time& ts) {
  T_f_g_ = T_f_g;
  T_f_b_start_ = T_f_b;
  ts_T_f_g_ = ts;
  sensing_ready_ = true;

  // Compute distance from goal to start
  Pose3 dT_start_goal = T_f_b_start_.inverse() * T_f_g_;
  distance_start_to_goal_ = std::hypot(dT_start_goal.translation().y(), dT_start_goal.translation().x());

  // Reset failure timer
  ts_failure_ = ts_T_f_b_;
  last_progress_ = 0.0;
  ts_last_progress_ = ts_T_f_b_;

  state_ = State::EXECUTING;
}

// Set state (pose + twist)
void BaseLocalPlanner::setPoseInFixed(const Pose3& T_f_b, const Time& ts) {
  T_f_b_ = T_f_b;
  ts_T_f_b_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setVelocityInBase(const Twist& b_v, const Time& ts) {
  b_v_ = b_v;
  ts_b_v_ = ts;
  sensing_ready_ = true;
}

std::string BaseLocalPlanner::stateToStr(BaseLocalPlanner::State state) {
  switch (state) {
    case BaseLocalPlanner::State::NOT_READY:
      return "NOT_READY";
    case BaseLocalPlanner::State::FINISHED:
      return "FINISHED";
    case BaseLocalPlanner::State::EXECUTING:
      return "EXECUTING";
    case BaseLocalPlanner::State::FAILURE:
      return "FAILURE";
    default:
      return "INVALID_STATE";
  }
}

}  // namespace field_local_planner
