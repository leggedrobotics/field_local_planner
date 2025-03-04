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
#include <rmp/models/DifferentialModel.hpp>

#include <cmath>
#include <fstream>
#include <iostream>

namespace gtsam {

DifferentialModel::DifferentialModel(double theta) : theta_(theta) {
  // Precompute model
  J_ << cos(theta_), 0.0, sin(theta_), 0.0, 0.0, 1.0;
}

DifferentialModel::DifferentialModel(const Pose2& state) : DifferentialModel(state.theta()) {}

Vector3 DifferentialModel::apply(const Vector2& acceleration, OptionalJacobian<3, 1> Hmodel, OptionalJacobian<3, 2> Hacc) const {
  // Fill Jacobian
  // TODO: Implement
  if (Hmodel) {
    *Hmodel << 1.0, 1.0, 1.0;
  }
  if (Hacc) {
    *Hacc = J_;
  }

  // Transform acceleration
  Vector3 output = J_ * acceleration;

  return output;
}

std::ostream& operator<<(std::ostream& os, const DifferentialModel& model) {
  os << "theta: " << model.theta();
  return os;
}

/* ************************************************************************* */
void DifferentialModel::print(const std::string& s) const {
  gtsam::print((Vector)vector(), s);
}

/* ************************************************************************* */
bool DifferentialModel::equals(const DifferentialModel& model, double tol) const {
  return (std::fabs(theta_ - model.theta_) < tol);
}

}  // namespace gtsam