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
#include <ros/console.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <iostream>

#include <field_local_planner_base/basic_types.hpp>

namespace field_local_planner {
namespace utils {

#define UPDATE_COMMON_PARAMS(VAR) utils::assignAndPrintDiff(#VAR, p.VAR, config.VAR);

//-------------------------------------------------------------------------------------------------
// Parameter reading
//-------------------------------------------------------------------------------------------------
template <typename T>
T getParameter(const ros::NodeHandle& nh, const std::string& param) {
  T value;
  if (!nh.getParam(param, value)) {
    ROS_ERROR_STREAM("Could not read parameter " << param);
    exit(-1);
  }
  ROS_INFO_STREAM(param << ": " << (value));

  return value;
}

template <typename T>
T getParameterDefault(const ros::NodeHandle& nh, const std::string& param, const T& default_value) {
  T value = default_value;
  if (!nh.getParam(param, value)) {
    ROS_WARN_STREAM(param << ": " << (value) << "(default)");
  } else {
    ROS_INFO_STREAM(param << ": " << (value));
  }
  return value;
}

template <typename T>
std::vector<T> getParameterVector(const ros::NodeHandle& nh, const std::string& param) {
  std::vector<T> vector;

  XmlRpc::XmlRpcValue xml_values;
  nh.getParam(param, xml_values);
  if (xml_values.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM(param << " parameter must be a list.");
    exit(-1);
  } else {
    for (int i = 0; i < xml_values.size(); i++) {
      vector.push_back(static_cast<T>(xml_values[i]));
    }
  }

  return vector;
}

template <typename T>
static void assignAndPrintDiff(std::string var_name, T& var, const T& new_val) {
  if (var != new_val) {
    ROS_WARN_STREAM("[" << var_name << "] changed from " << static_cast<T>(var) << " to " << static_cast<T>(new_val));
  }
  var = new_val;
}

//-------------------------------------------------------------------------------------------------
// Data converters
//-------------------------------------------------------------------------------------------------

// To GTSAM types
static inline Pose3 toPose3(const geometry_msgs::Pose& pose_msg) {
  Eigen::Isometry3d eigen_pose;
  tf2::fromMsg(pose_msg, eigen_pose);
  return Pose3(eigen_pose.matrix());
}

static inline Twist toTwist(const geometry_msgs::Twist& twist_msg) {
  Twist twist = Twist::Zero();

  // We use GTSAM's convention with orientation-then-position (i.e, angular velocity and then linear)
  twist(0) = twist_msg.angular.x;
  twist(1) = twist_msg.angular.y;
  twist(2) = twist_msg.angular.z;
  twist(3) = twist_msg.linear.x;
  twist(4) = twist_msg.linear.y;
  twist(5) = twist_msg.linear.z;

  return twist;
}

static inline Time toTimeStamp(const ros::Time& t) {
  // Convert to microseconds
  return (int64_t)floor(t.toNSec() / 1000);
}

static inline ros::Time toRosTime(const Time& t) {
  uint32_t sec = (uint32_t)floor(t * 1e-6);
  uint32_t nsec = (uint32_t)((t - sec * 1e6) * 1e-3);
  return ros::Time(sec, nsec);
}

// To ROS types
static inline geometry_msgs::Twist toTwistMsg(const Twist& twist) {
  geometry_msgs::Twist twist_msg;

  // We use GTSAM's convention with orientation-then-position (i.e, angular velocity and then linear)
  twist_msg.angular.x = twist(0);
  twist_msg.angular.y = twist(1);
  twist_msg.angular.z = twist(2);
  twist_msg.linear.x = twist(3);
  twist_msg.linear.y = twist(4);
  twist_msg.linear.z = twist(5);

  return twist_msg;
}

static inline geometry_msgs::Pose toPoseMsg(const Pose3& pose) {
  geometry_msgs::Pose pose_msg;
  Eigen::Isometry3d eigen_pose(pose.matrix());
  return tf2::toMsg(eigen_pose);
}

static inline field_local_planner_msgs::Status toStatusMsg(const BaseLocalPlanner::Status& status) {
  field_local_planner_msgs::Status status_msg;
  status_msg.state = status.state;
  status_msg.progress = status.progress;
  status_msg.progress_delta = status.progress_delta;
  status_msg.distance_to_goal = status.distance_to_goal;
  status_msg.orientation_to_goal = status.orientation_to_goal;
  status_msg.goal_reached = status.state == BaseLocalPlanner::State::FINISHED;
  status_msg.header.stamp = ros::Time::now();

  return status_msg;
}

static inline nav_msgs::Path toPathMsg(const Path& path, const std::string& frame) {
  ros::Time now = ros::Time::now();

  nav_msgs::Path path_msg;
  path_msg.header.stamp = now;
  path_msg.header.frame_id = frame;
  for (auto p : path) {
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = now;
    ps.pose = toPoseMsg(p);
    path_msg.poses.push_back(ps);
  }
  return path_msg;
}

static inline geometry_msgs::TransformStamped toTransformStamped(const Pose3& pose, const std::string& parent, const std::string& child,
                                                                 const ros::Time& stamp) {
  Eigen::Isometry3d eigen_pose(pose.matrix());
  geometry_msgs::TransformStamped transform = tf2::eigenToTransform(eigen_pose);
  transform.header.stamp = stamp;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  return transform;
}

//-------------------------------------------------------------------------------------------------
// Other
//-------------------------------------------------------------------------------------------------

const inline int64_t getHeaderUTime(const std_msgs::Header& header) {
  return (int64_t)floor(header.stamp.toNSec() / 1000);
}

}  // namespace utils
}  // namespace field_local_planner