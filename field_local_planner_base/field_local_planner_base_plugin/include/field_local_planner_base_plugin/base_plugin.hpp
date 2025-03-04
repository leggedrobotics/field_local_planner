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
#include <field_local_planner_base_plugin/BaseConfig.h>
#include <field_local_planner_msgs/MoveToAction.h>
#include <field_local_planner_msgs/Status.h>
#include <field_local_planner_base/base_local_planner.hpp>
#include <field_local_planner_base/basic_types.hpp>
#include <field_local_planner_base/utils.hpp>
#include <field_local_planner_base_plugin/utils.hpp>

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <memory>

namespace field_local_planner {

class BasePlugin {
 protected:
  BasePlugin() {}

 public:
  // This method needs to be defined by the other plugins
  virtual std::string getName() = 0;
  virtual void loadParameters(ros::NodeHandle& nh) = 0;
  virtual void setupRos(ros::NodeHandle& nh) = 0;
  virtual void publishVisualizations() = 0;

  // Initializes the plugin
  void initialize(ros::NodeHandle& nh);
  void loadBaseParameters(ros::NodeHandle& nh);
  void setupBaseRos(ros::NodeHandle& nh);
  bool execute(const ros::Time& stamp, geometry_msgs::Twist& twist_msg, nav_msgs::Path& path, field_local_planner_msgs::Status& status_msg);

 private:
  // Callbacks
  // Helpers
  void setPose(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header = std_msgs::Header());
  void setVelocity(const geometry_msgs::Twist& twist_msg, const std_msgs::Header& header = std_msgs::Header());
  void setGoal(const geometry_msgs::Pose& goal_msg, const std_msgs::Header& header = std_msgs::Header());

  // State callbacks
  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
  void twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& odo_msg);

  // Other callbacks
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg);
  void joyTwistCallback(const geometry_msgs::TwistConstPtr& twist_msg);
  void executeActionCB(const field_local_planner_msgs::MoveToGoalConstPtr &goal);
  void moveToRequestActionHandler();
  void preemptActionHandler();
  void dynamicReconfigureCallback(BaseConfig& config, uint32_t level);

  // Sensor callbacks
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void gridMapCallback(const grid_map_msgs::GridMap& gid_map_msg);
  void imageRgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  void imageRgbdCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
                         const sensor_msgs::CameraInfoConstPtr& info_msg);
  void imageDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  // Publishers
  void publishTransform(const Pose3& T_parent_child, const std::string& parent, const std::string& child,
                        const ros::Time& stamp = ros::Time::now());
  void publishStaticTransform(const Pose3& T_parent_child, const std::string& parent, const std::string& child,
                              const ros::Time& stamp = ros::Time::now());

  void publishCurrentGoal(const Pose3& T_fixed_goal, const ros::Time& stamp = ros::Time::now());
  void publishCurrentBase(const Pose3& T_fixed_base, const ros::Time& stamp = ros::Time::now());
  void publishPath(const Path& path);
  void publishStatus(const BaseLocalPlanner::Status& status);
  void publishTwist(const Twist& twist, const ros::Time& stamp = ros::Time::now());
  void publishZeroTwist();

  // Utils
  Pose3 queryTransform(const std::string& parent, const std::string& child, const ros::Time& stamp = ros::Time::now());
  bool isValidFrame(const std::string& frame) const;
  void getPointCloudFromGridMap(const grid_map::GridMap& grid_map, const ros::Time& stamp, pcl::PointCloud<PointType>::Ptr& cloud,
                                Pose3& T_f_s);
  void printStateInfo(const BaseLocalPlanner::State& new_state);

  // Base inversion
  Pose3 getBaseInversionTransform() const;
  Pose3 fixBaseInversion(const Pose3& T_f_b, const std::string& frame) const;
  Twist fixBaseInversion(const Twist& b_v) const;

 protected:
  std::shared_ptr<BaseLocalPlanner> local_planner_;  // The actual local planner
  ros::NodeHandle nh_local_planner_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;

  // Subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber rgb_image_sub_;
  ros::Subscriber rgbd_image_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber grid_map_sub_;

  ros::Subscriber goal_sub_;
  ros::Subscriber joy_twist_sub_;

  // Publishers
  ros::Publisher current_goal_pub_;
  ros::Publisher current_base_pub_;
  ros::Publisher output_twist_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  std::string output_twist_type_;

  // Action server
  using ActionServer = actionlib::SimpleActionServer<field_local_planner_msgs::MoveToAction>;
  std::shared_ptr<ActionServer> action_server_;
  field_local_planner_msgs::MoveToResult result_;
  field_local_planner_msgs::MoveToFeedback feedback_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<BaseConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<BaseConfig>::CallbackType dynamic_reconfigure_callback_;

  // Frames
  bool base_inverted_;       // To invert the base 180 deg
  std::string fixed_frame_;  // usually 'odom' frame
  std::string base_frame_;   // usually 'base' frame
  std::vector<std::string> valid_goal_frames_;

  // Point cloud utils
  pcl::VoxelGrid<PointType> voxel_filter_;
  bool grid_map_to_cloud_;
  double grid_map_to_cloud_range_;
  double grid_map_to_cloud_filter_size_;

  // Output
  ros::Time last_pose_stamp_;
  BaseLocalPlanner::State last_state_;
  Pose3 T_f_g_;  // Goal in fixed frame
};

}  // namespace field_local_planner
