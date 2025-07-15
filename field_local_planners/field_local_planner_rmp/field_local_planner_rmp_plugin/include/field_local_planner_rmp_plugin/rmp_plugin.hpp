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
#include <field_local_planner_rmp_plugin/RmpConfig.h>
#include <field_local_planner_base_plugin/base_plugin.hpp>
#include <field_local_planner_rmp/rmp_local_planner.hpp>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace field_local_planner {

class RmpPlugin : public BasePlugin {
 public:
  RmpPlugin();

  std::string getName() { return "rmp"; };
  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void publishVisualizations();

  void dynamicReconfigureCallback(RmpConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  using DynParamServer = dynamic_reconfigure::Server<RmpConfig>;
  using DynParamCallback = dynamic_reconfigure::Server<RmpConfig>::CallbackType;
  std::shared_ptr<DynParamServer> dynamic_reconfigure_server_;
  DynParamCallback dynamic_reconfigure_callback_;

  // Visualization publisher
  ros::Publisher control_points_pub_;
};

}  // namespace field_local_planner