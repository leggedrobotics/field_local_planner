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
#include <field_local_planner_base/base_local_planner.hpp>
#include <field_local_planner_base/utils.hpp>

// RMP stuff
#include <gtsam/base/Vector.h>
#include <rmp/rmp.hpp>

#include <boost/any.hpp>
#include <map>

namespace field_local_planner {

class Rmp : public BaseLocalPlanner {
 public:
  struct RmpParameters {
    std::string name;
    double weight;
    double gain;
    Vector3 color;
    std::string metric_type;
    double metric_offset;
    double metric_steepness;
  };

  struct Parameters {
    double robot_clearance = 0.1;
    double sphere_radius_factor = 1.0;
    double integration_time = 1.0;
    std::map<std::string, RmpParameters> rmp_parameters;
  };

  struct ControlPoint {
    std::string id;  // ID of the control point
    double radius;   // Radius for the collision sphere
    double inflated_radius;
    Vector2 position;                       // Position in the base frame
    Vector2 point_factor;                   // Factor used to scale the robot size and get the position
    Vector3 color;                          // color for visualization
    std::vector<std::string> affected_by;   // RMPs that affect the control point
    std::map<std::string, rmp::Rmp3> rmps;  // RMPs used for visualization
  };

  using ControlPoints = std::vector<Rmp::ControlPoint>;

 public:
  // Default constructor
  Rmp();
  // Interface to load parameters
  void setParameters(const Parameters& parameters);
  // Interface to get parameters
  Parameters getParameters() const;
  // COmputes the output twist
  Twist computeTwist();

  // Computes a path for visualization
  Path computePath();

  // Utils
  // Adds a control point
  void setControlPoints(const ControlPoints& control_points);
  // Gets the control points
  ControlPoints getControlPoints() const;
  // Gets a list of available RMPs
  std::vector<std::string> getAvailableRmps();

 private:
  // Solves the RMP optimization problem
  void computeOptimalAcceleration();

  // Helpers to make specific RMPs
  rmp::Rmp3 makeGeodesicGoalPolicy(ControlPoint& cp);
  rmp::Rmp3 makeGeodesicHeadingPolicy(ControlPoint& cp);
  rmp::Rmp3 makeGoalPositionPolicy(ControlPoint& cp);
  rmp::Rmp3 makeGoalOrientationPolicy(ControlPoint& cp);
  rmp::Rmp3 makeVelocityHeadingPolicy(ControlPoint& cp);
  rmp::Rmp3 makeDampingPolicy(ControlPoint& cp);
  rmp::Rmp2 makeSdfObstaclePolicy(ControlPoint& cp);
  rmp::Rmp2 makeSdfObstacleDampingPolicy(ControlPoint& cp);
  rmp::Rmp3 makeRegularizationPolicy(ControlPoint& cp);

  // Helper to query layers of the grid map
  void getGradientsFromGridMap(const std::string& layer, const Pose2& T_m_b_query, double& distance, Vector2& grad_in_base);

  // Convert Rmp2 to Rmp3 for visualization purposes
  rmp::Rmp3 convertToRmp3(const rmp::Rmp2& rmp);

 protected:
  Parameters parameters_;

  // Other internal variables
  // Internal
  Twist optimal_twist_;  // 6 DoF
  Vector3 velocity_2d_;
  Vector3 optimal_velocity_;
  Vector3 optimal_acc_;
  Matrix3 optimal_metric_;

  Pose3 T_m_b_;  // Base in map frame

  Pose2 T_m_b_SE2_;  // Pose of robot in map frame
  Pose2 T_f_b_SE2_;  // Pose of robot in fixed frame
  Pose2 T_f_g_SE2_;  // Pose of goal in fixed frame

  // Control points stuff
  std::vector<ControlPoint> control_points_;
};

}  // namespace field_local_planner
