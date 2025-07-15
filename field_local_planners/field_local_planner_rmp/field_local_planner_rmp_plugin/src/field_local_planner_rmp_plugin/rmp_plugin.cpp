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
#include <pluginlib/class_list_macros.h>
#include <field_local_planner_rmp_plugin/rmp_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(field_local_planner::RmpPlugin, field_local_planner::BasePlugin)

namespace field_local_planner {

RmpPlugin::RmpPlugin() : BasePlugin() {
  local_planner_ = std::make_shared<Rmp>();
}

void RmpPlugin::loadParameters(ros::NodeHandle& nh) {
  // Get base parameters for some computations
  BaseLocalPlanner::Parameters bp = local_planner_->getBaseParameters();

  Rmp::Parameters p;
  p.sphere_radius_factor = utils::getParameterDefault(nh, "sphere_radius_factor", 1.0);
  p.integration_time = utils::getParameterDefault(nh, "integration_time", 1.0);

  // Read RMP parameters
  for (auto rmp_name : std::dynamic_pointer_cast<Rmp>(local_planner_)->getAvailableRmps()) {
    Rmp::RmpParameters rmp_p;
    rmp_p.name = rmp_name;
    rmp_p.weight = utils::getParameterDefault(nh, "policies/" + rmp_p.name + "/weight", 1.0);
    rmp_p.gain = utils::getParameterDefault(nh, "policies/" + rmp_p.name + "/gain", 1.0);
    rmp_p.metric_type = utils::getParameterDefault<std::string>(nh, "policies/" + rmp_p.name + "/metric/type", "sigmoid");
    rmp_p.metric_offset = utils::getParameterDefault(nh, "policies/" + rmp_p.name + "/metric/offset", 1.0);
    rmp_p.metric_steepness = utils::getParameterDefault(nh, "policies/" + rmp_p.name + "/metric/steepness", 1.0);

    std::vector<double> color = utils::getParameterVector<double>(nh, "policies/" + rmp_p.name + "/color");
    rmp_p.color(0) = color[0];
    rmp_p.color(1) = color[1];
    rmp_p.color(2) = color[2];

    p.rmp_parameters[rmp_name] = rmp_p;
  }
  std::dynamic_pointer_cast<Rmp>(local_planner_)->setParameters(p);

  // Prepare control points
  XmlRpc::XmlRpcValue control_points_list;
  nh.getParam("control_points", control_points_list);

  Rmp::ControlPoints control_points;

  for (size_t i = 0; i < control_points_list.size(); ++i) {
    XmlRpc::XmlRpcValue node = control_points_list[i];

    Rmp::ControlPoint cp;
    cp.id = static_cast<std::string>(node["id"]);
    cp.radius = static_cast<double>(node["radius"]);
    cp.inflated_radius = cp.radius * p.sphere_radius_factor;
    cp.point_factor(0) = static_cast<double>(node["point_factor"][0]);
    cp.point_factor(1) = static_cast<double>(node["point_factor"][1]);
    cp.position(0) = cp.point_factor(0) * bp.robot_length * 0.5;
    cp.position(1) = cp.point_factor(1) * bp.robot_width * 0.5;
    cp.color(0) = static_cast<double>(node["color"][0]);
    cp.color(1) = static_cast<double>(node["color"][1]);
    cp.color(2) = static_cast<double>(node["color"][2]);

    XmlRpc::XmlRpcValue affected_by_list = node["affected_by"];
    for (size_t j = 0; j < affected_by_list.size(); ++j) {
      cp.affected_by.push_back(static_cast<std::string>(affected_by_list[j]));
    }
    control_points.push_back(cp);
  }
  std::dynamic_pointer_cast<Rmp>(local_planner_)->setControlPoints(control_points);
}

void RmpPlugin::setupRos(ros::NodeHandle& nh) {
  dynamic_reconfigure_server_ = std::make_shared<DynParamServer>(nh);
  dynamic_reconfigure_callback_ = boost::bind(&RmpPlugin::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(dynamic_reconfigure_callback_);

  control_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("control_points", 10);
}

void RmpPlugin::dynamicReconfigureCallback(RmpConfig& config, uint32_t level) {
#define UPDATE_RMP_PARAMS(RMP, VAR) utils::assignAndPrintDiff(#RMP "_" #VAR, p.rmp_parameters[#RMP].VAR, config.RMP##_##VAR);

#define UPDATE_RMP(RMP)          \
  UPDATE_RMP_PARAMS(RMP, weight) \
  UPDATE_RMP_PARAMS(RMP, gain)   \
  // UPDATE_RMP_PARAMS(RMP, metric_type)   \
  // UPDATE_RMP_PARAMS(RMP, metric_offset) \
  // UPDATE_RMP_PARAMS(RMP, metric_steepness)

  // Get base parameters for some computations
  BaseLocalPlanner::Parameters bp = local_planner_->getBaseParameters();

  Rmp::Parameters p = std::dynamic_pointer_cast<Rmp>(local_planner_)->getParameters();
  Rmp::ControlPoints cps = std::dynamic_pointer_cast<Rmp>(local_planner_)->getControlPoints();

  // RMP parameters
  UPDATE_COMMON_PARAMS(sphere_radius_factor)
  UPDATE_COMMON_PARAMS(integration_time)

  UPDATE_RMP(geodesic_goal)
  UPDATE_RMP(geodesic_heading)
  UPDATE_RMP(goal_position)
  UPDATE_RMP(goal_orientation)
  UPDATE_RMP(velocity_heading)
  UPDATE_RMP(damping)
  UPDATE_RMP(sdf_obstacle)
  UPDATE_RMP(sdf_obstacle_damping)
  UPDATE_RMP(regularization)

  // Update control points
  for (auto& cp : cps) {
    cp.inflated_radius = cp.radius * p.sphere_radius_factor;
    cp.position(0) = cp.point_factor(0) * bp.robot_length * 0.5;
    cp.position(1) = cp.point_factor(1) * bp.robot_width * 0.5;
  }

  std::dynamic_pointer_cast<Rmp>(local_planner_)->setParameters(p);
  std::dynamic_pointer_cast<Rmp>(local_planner_)->setControlPoints(cps);
}

void RmpPlugin::publishVisualizations() {
  Rmp::ControlPoints control_points = std::dynamic_pointer_cast<Rmp>(local_planner_)->getControlPoints();

  int i = 0;
  visualization_msgs::MarkerArray cp_vis;
  for (auto cp : control_points) {
    // Control point marker
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = BasePlugin::base_frame_;
      marker.ns = "control_point";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = cp.position(0);
      marker.pose.position.y = cp.position(1);
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2 * cp.radius;
      marker.scale.y = 2 * cp.radius;
      marker.scale.z = 2 * cp.radius;
      marker.color.r = cp.color(0);
      marker.color.g = cp.color(1);
      marker.color.b = cp.color(2);
      marker.color.a = 0.2;
      // Add marker
      cp_vis.markers.push_back(marker);
    }

    for (auto rmp : cp.rmps) {
      std::string name = rmp.first;
      rmp::Rmp3 policy = rmp.second;

      // Accelerations
      {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = BasePlugin::base_frame_;
        marker.ns = "acc_" + name;
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cp.position(0);
        marker.pose.position.y = cp.position(1);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = policy.color()(0);
        marker.color.g = policy.color()(1);
        marker.color.b = policy.color()(2);
        marker.color.a = 1.0;
        geometry_msgs::Point p1;
        p1.x = 0.0;
        p1.y = 0.0;
        p1.z = 0.0;
        geometry_msgs::Point p2;
        p2.x = policy.acceleration()(0) * 0.05 * policy.weight();
        p2.y = policy.acceleration()(1) * 0.05 * policy.weight();
        p2.z = 0.0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        // Add marker
        cp_vis.markers.push_back(marker);
      }

      // Angular accelerations
      // {
      //   visualization_msgs::Marker marker;
      //   marker.header = header;
      //   marker.header.frame_id = BasePlugin::base_frame_;
      //   marker.ns = "angular_acc_" + acc.id_;
      //   marker.id = i;
      //   marker.type = visualization_msgs::Marker::ARROW;
      //   marker.action = visualization_msgs::Marker::ADD;
      //   marker.pose.position.x = cp.position(0);
      //   marker.pose.position.y = cp.position(1);
      //   marker.pose.position.z = 0.0;
      //   marker.pose.orientation.x = 0.0;
      //   marker.pose.orientation.y = 0.0;
      //   marker.pose.orientation.z = 0.0;
      //   marker.pose.orientation.w = 1.0;
      //   marker.scale.x = 0.04;
      //   marker.scale.y = 0.07;
      //   marker.scale.z = 0.07;
      //   marker.color.r = policy.color()(0);
      //   marker.color.g = policy.color()(1);
      //   marker.color.b = policy.color()(2);
      //   marker.color.a = 1.0;
      //   geometry_msgs::Point p1;
      //   p1.x = 0.0;
      //   p1.y = 0.0;
      //   p1.z = 0.0;
      //   geometry_msgs::Point p2;
      //   p2.x = 0.0;
      //   p2.y = acc.angular_acc_ * 0.05 * policy.weight();
      //   p2.z = 0.0;
      //   marker.points.push_back(p1);
      //   marker.points.push_back(p2);
      //   // Add marker
      //   cp_vis.markers.push_back(marker);
      // }

      // Metric
      {
        visualization_msgs::Marker marker;
        // We first need to compute the eigenvectors and eigenvalues (based on supereight atlas)
        Eigen::Vector2d vis_eigenvalues(Eigen::Vector2d::Zero());
        Eigen::Matrix2d vis_eigenvectors(Eigen::Matrix2d::Zero());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> vis_solver(policy.metric().block(0, 0, 2, 2));

        if (vis_solver.info() == Eigen::Success) {
          vis_eigenvalues = vis_solver.eigenvalues();
          vis_eigenvectors = vis_solver.eigenvectors();
        } else {
          ROS_WARN_STREAM("Couldn't compute eigenvectors for metric of acc [" << name << "]");
          continue;
        }

        // Create rotation matrix from eigenvectors
        if (vis_eigenvectors.determinant() < 0) {
          vis_eigenvectors.col(1) *= -1;  // flip the direction of one axis
        }

        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        rot.block(0, 0, 2, 2) = vis_eigenvectors;
        Quaternion q(rot);
        q.normalize();

        // Create pose from rotation matrix and control point coords
        Pose3 pose(Rot3(q), Vector3(cp.position(0), cp.position(1), 0.0));
        Eigen::Isometry3d eigen_pose(pose.matrix());
        geometry_msgs::Pose metric_pose = tf2::toMsg(eigen_pose);

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = BasePlugin::base_frame_;
        marker.ns = "metric_" + name;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = metric_pose;

        double s1 = std::sqrt(vis_eigenvalues[0]);
        double s2 = std::sqrt(vis_eigenvalues[1]);
        s1 = std::isfinite(s1) ? s1 : 0.1;
        s2 = std::isfinite(s2) ? s2 : 0.1;
        double normalizer = 1.0;  // std::max(s1, s2);
        s1 = std::max(s1 / normalizer, 0.1);
        s2 = std::max(s2 / normalizer, 0.1);

        marker.scale.x = s1;
        marker.scale.y = s2;
        marker.scale.z = 0.02;
        marker.color.r = policy.color()(0);
        marker.color.g = policy.color()(1);
        marker.color.b = policy.color()(2);
        marker.color.a = 0.2;

        // Add marker
        cp_vis.markers.push_back(marker);
      }

      // Update ids
      i++;
    }
  }

  // Publish
  control_points_pub_.publish(cp_vis);
}

}  // namespace field_local_planner
