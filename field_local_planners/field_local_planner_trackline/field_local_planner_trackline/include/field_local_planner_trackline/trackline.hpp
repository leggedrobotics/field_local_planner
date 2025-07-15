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

namespace field_local_planner {

class Trackline : public BaseLocalPlanner {
 public:
  struct Parameters {
    double angular_gain_p;
    double linear_gain_p;
  };

  // For the internal state machine
  enum class State {
    TURN_TO_GOAL = 0,         // First the robot will rotate to the goal
    FORWARD = 1,              // Move forward tracking a line
    TURN_TO_DESTINATION = 2,  // Rotate to match the final destination
    UNKNOWN = -1,
  };

 public:
  Trackline();
  void setParameters(const Parameters& p);
  Parameters getParameters() const;

  Twist computeTwist();
  Path computePath();

private:
  void stateTurnToGoal();
  void stateForward();
  void stateTurnToDestination();

 protected:
  Parameters parameters_;
  State state_;

  // Trackline directed velocity commands
  double trackline_linvel_x_, trackline_linvel_y_;

  // Goal directed velocity commands
  double goal_linvel_x_;
  double goal_linvel_y_;
  double goal_angvel_z_;
};

}  // namespace field_local_planner
