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
#include <gtest/gtest.h>

// Std lib
#include <stdio.h>
#include <chrono>
#include <fstream>
#include <iostream>

// RMP
#include <rmp/rmp.hpp>
// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/any.hpp>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace rmp;

TEST(UnitTests, MetricTest) {
  Matrix3 m1 = Metric3::make("constant");
  ASSERT_EQ(m1, Metric3::makeConstantMetric());

  Matrix3 m2 = Metric3::make("orthoprojector", 0.0, 0.0, 0.0, Vector3(1, 0, 0));
  Matrix3 m3 = Metric3::make("projector", 0.0, 0.0, 0.0, Vector3(1, 0, 0));
  ASSERT_EQ(m2, Matrix3::Identity() - m3);
}

TEST(UnitTests, RmpConstructorTest) {
  Rmp2 rmp(Vector2(1, 2), Matrix2::Identity() * 2.0, 0.5, "test_rmp", Vector3(1.0, 0.5, 0.1));
  const double tol = 1e-10;

  ASSERT_EQ(1, rmp.acceleration()(0));
  ASSERT_EQ(2, rmp.acceleration()(1));

  ASSERT_NEAR(2.0, rmp.metric()(0, 0), tol);
  ASSERT_NEAR(2.0, rmp.metric()(1, 1), tol);

  ASSERT_NEAR(0.5, rmp.weight(), tol);

  ASSERT_EQ("test_rmp", rmp.name());

  ASSERT_NEAR(1.0, rmp.color()(0), tol);
  ASSERT_NEAR(0.5, rmp.color()(1), tol);
  ASSERT_NEAR(0.1, rmp.color()(2), tol);
}

TEST(UnitTests, GoalRmpTest) {
  // Create RMP problem
  rmp::Problem problem;

  // Create variable
  Symbol x_ddot('x');
  Vector3_ acc(x_ddot);

  // State
  Pose2 state(0, 0, 0);
  Vector3 velocity(0, 0, 0);

  // Create RMP
  Pose2 goal(1, 0, 0);
  problem.addRmp(acc, Rmp3(MotionPolicy::makeGoalPosePolicy(state, goal, 1.0), Metric3::makeConstantMetric()));

  // Create initial values
  Values initial_acc;
  initial_acc.insert(x_ddot, Vector3(0.0, 0.0, 0.0));

  // Solve
  Values accelerations = problem.solve(initial_acc, false);
  Vector3 opt_acc = accelerations.at<Vector3>(x_ddot);

  ASSERT_NEAR(opt_acc(0), 1.0, 1e-3);
  ASSERT_NEAR(opt_acc(1), 0.0, 1e-3);
  ASSERT_NEAR(opt_acc(2), 0.0, 1e-3);
}

TEST(UnitTests, SolversTest) {
  // Create RMP problem
  rmp::Problem problem;

  // Create variable
  Symbol x_ddot('x');
  Vector3_ acc(x_ddot);

  // State
  Pose2 state(0, 0, 0);
  Vector3 velocity(0, 0, 0);

  // Create RMP
  Pose2 goal(1, 0, 0);
  problem.addRmp(acc, Rmp3(MotionPolicy::makeGoalPosePolicy(state, goal, 1.0), Metric3::makeConstantMetric()));

  // Create initial values
  Values initial_acc;
  initial_acc.insert(x_ddot, Vector3(0.0, 0.0, 0.0));

  // Solve with GN
  rmp::Problem::Parameters gn_params;
  gn_params.algorithm = "GAUSS_NEWTON";
  Values gn_sol = problem.solve(initial_acc, true, gn_params);
  Vector3 gn_acc = gn_sol.at<Vector3>(x_ddot);

  // Solve with LM
  rmp::Problem::Parameters lm_params;
  lm_params.algorithm = "LM";
  Values lm_sol = problem.solve(initial_acc, true, lm_params);
  Vector3 lm_acc = lm_sol.at<Vector3>(x_ddot);

  // Solve with DogLeg
  rmp::Problem::Parameters dl_params;
  dl_params.algorithm = "DOGLEG";
  Values dl_sol = problem.solve(initial_acc, true, dl_params);
  Vector3 dl_acc = dl_sol.at<Vector3>(x_ddot);

  const double tol = 1e-6;
  ASSERT_NEAR(gn_acc(0), lm_acc(0), tol);
  ASSERT_NEAR(gn_acc(0), dl_acc(0), tol);

  ASSERT_NEAR(gn_acc(1), lm_acc(1), tol);
  ASSERT_NEAR(gn_acc(1), dl_acc(1), tol);

  ASSERT_NEAR(gn_acc(2), lm_acc(2), tol);
  ASSERT_NEAR(gn_acc(2), dl_acc(2), tol);
}

TEST(UnitTests, GoalRmpGainTest) {
  // Create RMP problem
  rmp::Problem problem;

  // Create variable
  Symbol x_ddot('x');
  Vector3_ acc(x_ddot);

  // State
  Pose2 state(0, 0, 0);
  Vector3 velocity(0, 0, 0);

  // Create RMP
  Pose2 goal(1, 0, 0);
  problem.addRmp(acc, Rmp3(MotionPolicy::makeGoalPosePolicy(state, goal, 10.0), Metric3::makeConstantMetric()));

  // Create initial values
  Values initial_acc;
  initial_acc.insert(x_ddot, Vector3(0.0, 0.0, 0.0));

  // Solve
  Values accelerations = problem.solve(initial_acc, false);
  Vector3 opt_acc = accelerations.at<Vector3>(x_ddot);

  ASSERT_NEAR(opt_acc(0), 10.0, 1e-3);
  ASSERT_NEAR(opt_acc(1), 0.0, 1e-3);
  ASSERT_NEAR(opt_acc(2), 0.0, 1e-3);
}

TEST(UnitTests, ControlPointTest) {
  // Create RMP problem
  rmp::Problem problem;

  // Create variable
  Symbol x_ddot = symbol_shorthand::X(0);
  Vector3_ acc(x_ddot);

  // Control Point
  ControlPoint2 cp(Vector2(0.0, 0.0));
  ControlPoint2_ cp_(cp);

  // Create RMP
  problem.addRmp(acc, Rmp3(Vector3(9.99, 0.06, 0), Vector3(1, 1, 0.001).asDiagonal()));
  problem.addRmp(acc, Rmp3(Vector3(0, 0, 0.06), Vector3(0.001, 0.001, 1.0).asDiagonal()));
  problem.addRmp(rmp::pullback(cp_, acc), Rmp2(Vector2(8.38, -0.83), Vector2(1, 1).asDiagonal()));

  // Create initial values
  Values initial_acc;
  initial_acc.insert(x_ddot, Vector3(0.0, 0.0, 0.0));

  // Solve
  Values accelerations = problem.solve(initial_acc, false);
  Vector3 opt_acc = accelerations.at<Vector3>(x_ddot);

  // Check that acceleration is finite
  ASSERT_TRUE(std::isfinite(opt_acc(0)));
  ASSERT_TRUE(std::isfinite(opt_acc(1)));
  ASSERT_TRUE(std::isfinite(opt_acc(2)));

  // Check that metric is finite
  ASSERT_TRUE(std::isfinite(problem.metric()(0, 0)));
  ASSERT_TRUE(std::isfinite(problem.metric()(1, 1)));
  ASSERT_TRUE(std::isfinite(problem.metric()(2, 2)));
}