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
#include <field_local_planner_trackline_plugin/TracklineConfig.h>
#include <field_local_planner_base_plugin/base_plugin.hpp>
#include <field_local_planner_trackline/trackline.hpp>

namespace field_local_planner {

class TracklinePlugin : public BasePlugin {
 public:
  TracklinePlugin();

  std::string getName() { return "trackline"; };
  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);
  void publishVisualizations();

  void dynamicReconfigureCallback(TracklineConfig& config, uint32_t level);

 private:
  // Dynamic reconfigure
  using DynParamServer = dynamic_reconfigure::Server<TracklineConfig>;
  using DynParamCallback = dynamic_reconfigure::Server<TracklineConfig>::CallbackType;
  std::shared_ptr<DynParamServer> dynamic_reconfigure_server_;
  DynParamCallback dynamic_reconfigure_callback_;
};

}  // namespace field_local_planner