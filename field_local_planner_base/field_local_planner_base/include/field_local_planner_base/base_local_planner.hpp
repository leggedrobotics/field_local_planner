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

#include <field_local_planner_base/basic_types.hpp>
#include <field_local_planner_base/profiler.hpp>
#include <field_local_planner_base/timer.hpp>
#include <field_local_planner_base/utils.hpp>

#include <grid_map_core/GridMap.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <string>

namespace field_local_planner {

using namespace gtsam;

class BaseLocalPlanner {
 public:
  struct Parameters {
    bool requires_sensing = true;
    bool base_inverted = false;
    bool differential_mode = false;
    double control_rate = 10.0;
    double robot_length = 1.0;
    double robot_width = 1.0;
    double robot_height = 1.0;
    double distance_to_goal_thr = 0.1;     // meters
    double orientation_to_goal_thr = 0.1;  // radians
    double max_linear_velocity_x = 1.0;    // m/s
    double max_linear_velocity_y = 1.0;    // m/s
    double max_angular_velocity_z = 1.0;   // rad/s
    double progress_threshold = 0.1;
    double failure_timeout_sec = 10;  // seconds
  };

  // Possible local planner states
  enum State {
    NOT_READY = 0,  // Initial state
    FINISHED = 1,   // Robot at goal. Do nothing
    EXECUTING = 2,  // Robot trying to reach the goal
    FAILURE = 3,    // Robot cannot make any progress, potentially a failure
  };

  struct Status {
    State state;
    double progress;
    double progress_delta;
    double distance_to_goal;
    double orientation_to_goal;
  };

  // This defines the output of the local planner
  struct Output {
    Twist twist;
    Path path;
    Status status;
  };

 public:
  // Compute the twist command - this function depends on the local planner
  virtual Twist computeTwist() = 0;
  virtual Path computePath() = 0;

  // Check failure - it may depend on the method
  bool checkFailure();

  // Constructor
  BaseLocalPlanner();

  // Parameters
  void setBaseParameters(const Parameters& parameters);
  Parameters getBaseParameters() const;

  // Main method to execute the local planner
  bool execute(const Time& ts, Output& output);
  void stop();

  // Interfaces for external data
  void setImageRgb(const cv::Mat& img, const Pose3& T_f_s, const Time& ts);
  void setImageRgbd(const cv::Mat& img, const Pose3& T_f_s, const Time& ts);
  void setImageDepth(const cv::Mat& img, const Pose3& T_f_s, const Time& ts);
  void setPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, const Pose3& T_f_s, const Time& ts);
  void setGridMap(const grid_map::GridMap& grid_map, const Pose3& T_f_s, const Time& ts);

  // Set state (pose + twist)
  void setPoseInFixed(const Pose3& T_f_b, const Time& ts);
  void setVelocityInBase(const Twist& b_v, const Time& ts);

  // Set single goal
  void setGoalInFixed(const Pose3& T_f_g, const Pose3& T_f_b, const Time& ts);

  std::string stateToStr(State state);
  Twist limitTwist(const Twist& twist);

 private:
  // Other steps
  State checkState();
  void computeDistanceAndOrientationToGoal();

 public:
  // Parameters
  Parameters base_parameters_;

 protected:
  State state_;

  // Timestamp
  Time last_ts_;

  // Possible sensor modalities and their sensor poses in the base frame
  cv::Mat image_rgb_;                            // RGB (color) image
  cv::Mat image_rgbd_;                           // RGB-D image
  cv::Mat image_depth_;                          // Depth image
  pcl::PointCloud<PointType>::Ptr point_cloud_;  // Point cloud
  grid_map::GridMap grid_map_;                   // Grid map
  Pose3 T_f_s_rgb_;                              // Pose of RGB camera in fixed frame
  Pose3 T_f_s_rgbd_;                             // Pose of RGB-D camera in fixed frame
  Pose3 T_f_s_depth_;                            // Pose of Depth camera in fixed frame
  Pose3 T_f_s_pc_;                               // Pose of point cloud in fixed frame
  Pose3 T_f_s_gm_;                               // Pose of grid map in fixed frame
  Time ts_rgb_;                                  // Timestamp of RGB
  Time ts_rgbd_;                                 // Timestamp of RGB-D
  Time ts_depth_;                                // Timestamp of Depth
  Time ts_pc_;                                   // Timestamp of point cloud
  Time ts_gm_;                                   // Timestamp of grid map

  // Robot state
  Pose3 T_f_b_;  // Pose of base in fixed frame
  Twist b_v_;    // Twist in base frame

  // Goal
  Pose3 T_f_b_start_;  // Pose when the goal was set
  Pose3 T_f_g_;        // Goal in fixed frame

  // Timestamps
  Time ts_T_f_b_;
  Time ts_b_v_;
  Time ts_T_f_g_;

  // Progress
  double progress_;
  double progress_delta_;
  double last_progress_;
  Time ts_last_progress_;

  Time ts_failure_;

  // Helpers
  Pose3 dT_b_g_;                   // Pose difference w.r.t goal
  Pose3 dT_b_start_;               // Pose differente w.r.t pose when goal was set
  double distance_to_goal_;        // distance w.r.t target position
  double orientation_to_goal_;     // orientation w.r.t target orientation
  double distance_to_start_;       // distance w.r.t starting position when goal was set
  double distance_start_to_goal_;  // distance from the starting position to the goal
  double orientation_to_start_;    // orientation w.r.t starting orientation when goal was set
  double heading_towards_goal_;    // How much the robot should rotate to point towards the goal

  // Other flags
  bool sensing_ready_;
};

}  // namespace field_local_planner