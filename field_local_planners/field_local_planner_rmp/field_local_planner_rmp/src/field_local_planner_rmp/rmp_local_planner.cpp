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
#include <field_local_planner_rmp/rmp_local_planner.hpp>

namespace field_local_planner {

Rmp::Rmp() : BaseLocalPlanner() {}

void Rmp::setParameters(const Rmp::Parameters& parameters) {
  parameters_ = parameters;
}

Rmp::Parameters Rmp::getParameters() const {
  return parameters_;
}

void Rmp::setControlPoints(const Rmp::ControlPoints& control_points) {
  control_points_ = control_points;
}

Rmp::ControlPoints Rmp::getControlPoints() const {
  return control_points_;
}

Twist Rmp::computeTwist() {
  // Get pose in map frame
  T_m_b_ = T_f_s_gm_.inverse() * T_f_b_;

  // Convert 3D poses to SE(2)
  T_f_b_SE2_ = utils::toSE2(T_f_b_);
  T_m_b_SE2_ = utils::toSE2(T_m_b_);
  T_f_g_SE2_ = utils::toSE2(T_f_g_);

  // Fill velocity 2d
  velocity_2d_(0) = b_v_(3);
  velocity_2d_(1) = b_v_(4);
  velocity_2d_(2) = b_v_(2);

  // Compute RMPs
  computeOptimalAcceleration();

  // Fill twist
  optimal_twist_(0) = 0.0;  // Angular x
  optimal_twist_(1) = 0.0;  // Angular y
  optimal_twist_(2) = optimal_velocity_.z();
  // Linear
  optimal_twist_(3) = optimal_velocity_.x();
  optimal_twist_(4) = optimal_velocity_.y();
  optimal_twist_(5) = 0.0;  // Linear z

  // Apply limits. We need to add it here (even though the base local planner already does it)
  // or the path will not be computed correctly
  optimal_twist_ = limitTwist(optimal_twist_);

  return optimal_twist_;
}

Path Rmp::computePath() {
  double time_horizon = 0.5;  // seconds
  double dt = 0.1;            // seconds

  Path path;
  for (double t = 0.0; t <= time_horizon; t += dt) {
    path.push_back(Pose3::Expmap(optimal_twist_ * t));
  }

  return path;
}

std::vector<std::string> Rmp::getAvailableRmps() {
  return std::vector<std::string>{"geodesic_goal", "geodesic_heading", "goal_position",        "goal_orientation", "velocity_heading",
                                  "damping",       "sdf_obstacle",     "sdf_obstacle_damping", "regularization"};
}

//-------------------------------------------------------------------------------------------------
// RMP optimization
//-------------------------------------------------------------------------------------------------

void Rmp::computeOptimalAcceleration() {
  // ------------------------------------------------------------------------------------
  // Initialize
  // ------------------------------------------------------------------------------------

  // Prepare free variable to store solution
  gtsam::Symbol acc_se2_key = gtsam::symbol_shorthand::X(0);
  gtsam::Symbol acc_diff_key = gtsam::symbol_shorthand::X(0);
  Vector3_ acc_se2(acc_se2_key);    // Full SE(2) acceleration
  Vector2_ acc_diff(acc_diff_key);  // Differentially constrained acceleration

  // Create general RMP problem
  rmp::Problem problem;

  // Create initial values
  // We use the last optimal acceleration
  Values initial_se2_acc;
  initial_se2_acc.insert(acc_se2_key, optimal_acc_);

  Values initial_diff_acc;
  initial_diff_acc.insert(acc_diff_key, Vector2(optimal_acc_(0), optimal_acc_(2)));

  // Create differential model (if required)
  DifferentialModel motion_model(0.0);
  rmp::DifferentialModel_ motion_model_(motion_model);

  if (base_parameters_.differential_mode) {
    acc_se2 = rmp::pullback(motion_model_, acc_diff);
  }

  // ------------------------------------------------------------------------------------
  // Add RMPs for control points
  // ------------------------------------------------------------------------------------
  for (size_t i = 0; i < control_points_.size(); i++) {
    ControlPoint& cp = control_points_.at(i);
    rmp::ControlPoint2_ cp_(ControlPoint2(cp.position));

    // std::cout << "Adding rmps to [" << cp.id.c_str() << "]" << std::endl;

    for (auto rmp_name : cp.affected_by) {
      if (rmp_name == "geodesic_goal") {
        rmp::Rmp3 policy = makeGeodesicGoalPolicy(cp);
        problem.addRmp(acc_se2, policy);
        cp.rmps[rmp_name] = policy;

      } else if (rmp_name == "geodesic_heading") {
        rmp::Rmp3 policy = makeGeodesicHeadingPolicy(cp);
        problem.addRmp(acc_se2, policy);
        cp.rmps[rmp_name] = policy;

      } else if (rmp_name == "goal_position") {
        rmp::Rmp3 policy = makeGoalPositionPolicy(cp);
        problem.addRmp(acc_se2, policy);
        cp.rmps[rmp_name] = policy;

      } else if (rmp_name == "goal_orientation") {
        rmp::Rmp3 policy = makeGoalOrientationPolicy(cp);
        problem.addRmp(acc_se2, policy);
        cp.rmps[rmp_name] = policy;

      } else if (rmp_name == "velocity_heading") {
        rmp::Rmp3 policy = makeVelocityHeadingPolicy(cp);
        problem.addRmp(acc_se2, policy);
        cp.rmps[rmp_name] = policy;

      } else if (rmp_name == "damping") {
        rmp::Rmp3 policy = makeDampingPolicy(cp);
        problem.addRmp(acc_se2, policy);
        cp.rmps[rmp_name] = policy;

      } else if (rmp_name == "sdf_obstacle") {
        rmp::Rmp2 policy = makeSdfObstaclePolicy(cp);
        problem.addRmp(rmp::pullback(cp_, acc_se2), policy);
        cp.rmps[rmp_name] = convertToRmp3(policy);

      } else if (rmp_name == "sdf_obstacle_damping") {
        rmp::Rmp2 policy = makeSdfObstacleDampingPolicy(cp);
        problem.addRmp(rmp::pullback(cp_, acc_se2), policy);
        cp.rmps[rmp_name] = convertToRmp3(policy);

      } else if (rmp_name == "regularization") {
        rmp::Rmp3 policy = makeRegularizationPolicy(cp);
        problem.addRmp(acc_se2, policy);
        cp.rmps[rmp_name] = policy;

      } else {
        std::cout << "WARNING: motion policy [" << rmp_name << "] not implemented" << std::endl;
      }

      // std::cout << cp.rmps[rmp_name] << std::endl;
    }
  }

  // ------------------------------------------------------------------------------------
  // Solve RMP general problem
  // ------------------------------------------------------------------------------------
  // profiler_ptr_->startEvent("1.3.2.solve_rmps");
  Values solution;
  bool valid_solution = true;
  try {
    if (base_parameters_.differential_mode) {
      solution = problem.solve(initial_diff_acc, true);
    } else {
      solution = problem.solve(initial_se2_acc, true);
    }
  } catch (const std::exception& e) {
    valid_solution = false;
    std::cout << "Error thrown during optimization of RMP problem:\n" << std::string(e.what()) << std::endl;
  }

  // Extract solution
  optimal_acc_ = Vector3::Zero();
  optimal_metric_ = Matrix3::Identity();

  if (valid_solution) {
    if (base_parameters_.differential_mode) {
      Vector2 optimal_acc_diff = solution.at<Vector2>(acc_diff_key);
      Matrix22 optimal_metric_diff = problem.metric();

      // Fill optimal acceleration
      optimal_acc_(0) = optimal_acc_diff(0);
      optimal_acc_(2) = optimal_acc_diff(1);

      // Fill optimal metric
      optimal_metric_ = Matrix33::Identity();
      optimal_metric_(0, 0) = optimal_metric_diff(0, 0);
      optimal_metric_(0, 2) = optimal_metric_diff(0, 1);
      optimal_metric_(2, 0) = optimal_metric_diff(1, 0);
      optimal_metric_(2, 2) = optimal_metric_diff(1, 1);

    } else {
      optimal_acc_ = solution.at<Vector3>(acc_se2_key);
      optimal_metric_ = problem.metric();
    }
  } else {
    std::cout << "Invalid solution" << std::endl;
  }

  // profiler_ptr_->endEvent("1.3.2.solve_rmps");

  // ------------------------------------------------------------------------------------
  // Integrate acceleration
  // ------------------------------------------------------------------------------------
  // std::cout << "Optimal acceleration: " << optimal_acc_.transpose() << std::endl;
  // std::cout << "Optimal metric:       " << optimal_metric_.diagonal().transpose() << std::endl;
  optimal_velocity_ = optimal_acc_ * parameters_.integration_time;
}

void Rmp::getGradientsFromGridMap(const std::string& layer, const Pose2& T_m_b_query, double& distance, Vector2& grad_in_base) {
  // Get distance to goal and gradient in fixed frame
  Vector2 base_position = T_m_b_query.translation();

  double grad_in_map_x, grad_in_map_y;
  try {
    distance = grid_map_.atPosition(layer, base_position);
    grad_in_map_x = grid_map_.atPosition(layer + "_gradient_x", base_position);
    grad_in_map_y = grid_map_.atPosition(layer + "_gradient_y", base_position);

  } catch (const std::out_of_range& oor) {
    std::cout << "Error while trying to access [" << layer << "] layer at " << base_position.transpose() << "\n Error: \n"
              << oor.what() << std::endl;
  }

  // Gradient is defined in the map frame, so we need to rotate it
  grad_in_base = T_m_b_query.rotation().inverse() * Vector2(grad_in_map_x, grad_in_map_y);
}

rmp::Rmp3 Rmp::makeGeodesicGoalPolicy(ControlPoint& cp) {
  const std::string rmp_name = "geodesic_goal";

  // Precompute geodesic gradients
  double distance = 0;
  Vector2 grad_in_base;
  getGradientsFromGridMap("geodesic", T_m_b_SE2_, distance, grad_in_base);

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector3 acc = rmp::MotionPolicy::makeGradientPositionPolicy(grad_in_base, params.gain);
  Matrix3 metric = rmp::Metric3::make(params.metric_type, params.metric_offset, params.metric_steepness,
                                      BaseLocalPlanner::distance_to_goal_, velocity_2d_);

  // Hack: decrease contribution of angular component in metric
  metric(2, 2) = 1e-3;

  // Return Riemannian Motion Policy
  return rmp::Rmp3(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp3 Rmp::makeGeodesicHeadingPolicy(ControlPoint& cp) {
  const std::string rmp_name = "geodesic_heading";

  // Get gradients
  double distance = 0;
  Vector2 grad_in_base;
  getGradientsFromGridMap("geodesic", T_m_b_SE2_, distance, grad_in_base);

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector3 acc = rmp::MotionPolicy::makeGradientOrientationPolicy(grad_in_base, params.gain);
  Matrix3 metric = rmp::Metric3::make(params.metric_type, params.metric_offset, params.metric_steepness,
                                      BaseLocalPlanner::distance_to_goal_, velocity_2d_);

  // Hack: decrease contribution of translational component in metric
  metric(0, 0) = 1e-3;
  metric(1, 1) = 1e-3;

  // Return Riemannian Motion Policy
  return rmp::Rmp3(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp3 Rmp::makeGoalPositionPolicy(ControlPoint& cp) {
  const std::string rmp_name = "goal_position";

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector3 acc = rmp::MotionPolicy::makeGoalPositionPolicy(T_f_b_SE2_, T_f_g_SE2_, params.gain);
  Matrix3 metric = rmp::Metric3::make(params.metric_type, params.metric_offset, params.metric_steepness,
                                      BaseLocalPlanner::distance_to_goal_, velocity_2d_);

  // Return Riemannian Motion Policy
  return rmp::Rmp3(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp3 Rmp::makeGoalOrientationPolicy(ControlPoint& cp) {
  const std::string rmp_name = "goal_orientation";

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector3 acc = rmp::MotionPolicy::makeGoalOrientationPolicy(T_f_b_SE2_, T_f_g_SE2_, params.gain);
  Matrix3 metric = rmp::Metric3::make(params.metric_type, params.metric_offset, params.metric_steepness,
                                      BaseLocalPlanner::distance_to_goal_, velocity_2d_);

  // Return Riemannian Motion Policy
  return rmp::Rmp3(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp3 Rmp::makeVelocityHeadingPolicy(ControlPoint& cp) {
  const std::string rmp_name = "velocity_heading";

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector3 acc = rmp::MotionPolicy::makeVelocityOrientationPolicy(velocity_2d_, Vector2(1.0, 0.0), params.gain);
  Matrix3 metric = rmp::Metric3::make(params.metric_type, params.metric_offset, params.metric_steepness,
                                      BaseLocalPlanner::distance_to_goal_, velocity_2d_);

  // Return Riemannian Motion Policy
  return rmp::Rmp3(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp3 Rmp::makeDampingPolicy(ControlPoint& cp) {
  const std::string rmp_name = "damping";

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector3 acc = rmp::MotionPolicy::makeDampingPolicy(velocity_2d_, params.gain);
  Matrix3 metric = rmp::Metric3::make(params.metric_type, params.metric_offset, params.metric_steepness,
                                      BaseLocalPlanner::distance_to_goal_, velocity_2d_);

  // Return Riemannian Motion Policy
  return rmp::Rmp3(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp2 Rmp::makeSdfObstaclePolicy(ControlPoint& cp) {
  const std::string rmp_name = "sdf_obstacle";

  // Get the pose of the control point in the map frame
  Pose2 T_m_cp_SE2_ = T_m_b_SE2_ * Pose2(0.0, cp.position);

  // This RMP is annoying since it requires 2d data
  Vector2 velocity = velocity_2d_.head<2>();

  // Get gradients
  double distance = 0;
  Vector2 grad_in_base;
  getGradientsFromGridMap("sdf", T_m_cp_SE2_, distance, grad_in_base);
  distance = distance - cp.inflated_radius;

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector2 acc = rmp::MotionPolicy::makeObstaclePolicy(grad_in_base, velocity, distance, params.gain);
  Matrix2 metric = rmp::Metric2::make(params.metric_type, params.metric_offset, params.metric_steepness, distance, grad_in_base);

  // Return Riemannian Motion Policy
  return rmp::Rmp2(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp2 Rmp::makeSdfObstacleDampingPolicy(ControlPoint& cp) {
  const std::string rmp_name = "sdf_obstacle_damping";

  // Get the pose of the control point in the map frame
  Pose2 T_m_cp_SE2_ = T_m_b_SE2_ * Pose2(0.0, cp.position);

  // This RMP is annoying since it requires 2d data
  Vector2 velocity = velocity_2d_.head<2>();

  // Get gradients
  double distance = 0;
  Vector2 grad_in_base;
  getGradientsFromGridMap("sdf", T_m_cp_SE2_, distance, grad_in_base);
  distance = distance - cp.inflated_radius;

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector2 acc = rmp::MotionPolicy::makeObstacleDampingPolicy(grad_in_base, velocity, distance, params.gain);
  Matrix2 metric = rmp::Metric2::make(params.metric_type, params.metric_offset, params.metric_steepness, distance, grad_in_base);

  // Return Riemannian Motion Policy
  return rmp::Rmp2(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp3 Rmp::makeRegularizationPolicy(ControlPoint& cp) {
  const std::string rmp_name = "regularization";

  // Create RMP
  RmpParameters params = parameters_.rmp_parameters[rmp_name];
  Vector3 acc = optimal_acc_;
  Matrix3 metric = rmp::Metric3::make(params.metric_type, params.metric_offset, params.metric_steepness,
                                      BaseLocalPlanner::distance_to_goal_, velocity_2d_);

  // Return Riemannian Motion Policy
  return rmp::Rmp3(acc, metric, params.weight, rmp_name, params.color);
}

rmp::Rmp3 Rmp::convertToRmp3(const rmp::Rmp2& rmp) {
  Vector3 acc = Vector3::Zero();
  acc.head<2>() = rmp.acceleration();

  Matrix3 metric = Matrix3::Zero();
  metric.block(0, 0, 2, 2) = rmp.metric();

  return rmp::Rmp3(acc, metric, rmp.weight(), rmp.name(), rmp.color());
}

}  // namespace field_local_planner