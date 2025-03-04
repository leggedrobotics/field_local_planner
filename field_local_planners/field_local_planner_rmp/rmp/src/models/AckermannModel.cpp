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
#include <rmp/models/AckermannModel.hpp>

#include <cmath>
#include <fstream>
#include <iostream>

namespace gtsam {

AckermannModel::AckermannModel(double v, double theta, double vehicleLength, double vehicleWidth, double distanceToVehicle)
    : v_(v), theta_(theta), vehicleLength_(vehicleLength), vehicleWidth_(vehicleWidth), distanceToCenter_(distanceToVehicle) {
  precomputeModel();
}

AckermannModel::AckermannModel(const Pose2& state, const Point3& velocity, double vehicleLength, double vehicleWidth,
                               double distanceToVehicle)
    : vehicleLength_(vehicleLength), vehicleWidth_(vehicleWidth), distanceToCenter_(distanceToVehicle) {
  theta_ = state.theta();
  v_ = velocity.head(2).norm();

  precomputeModel();
}

void AckermannModel::precomputeModel() {
  double xi = atan2(vehicleLength_, distanceToCenter_ + 0.5 * vehicleWidth_);
  double beta = atan2(0.5 * vehicleLength_, distanceToCenter_ + 0.5 * vehicleWidth_);

  J_ << cos(theta_), 0.0, sin(theta_), 0.0, sin(2.0 * beta) / vehicleWidth_, 4.0 * v_ * cos(2.0 * beta) / (3.0 * pow(cos(xi + 1), 2));

  // Fill output
  nonlinearTerm_ << 0.0, pow(v_, 2) / vehicleLength_ * cos(theta_) * sin(2 * beta), 0.0;
}

Vector3 AckermannModel::apply(const Vector2& acceleration, OptionalJacobian<3, 5> Hmodel, OptionalJacobian<3, 2> Hacc) const {
  // Fill Jacobian
  // TODO: Implement
  if (Hmodel) {
    *Hmodel << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  }
  if (Hacc) {
    *Hacc = J_;
  }

  // Transform acceleration
  Vector3 output = J_ * acceleration + nonlinearTerm_;

  return output;
}

std::ostream& operator<<(std::ostream& os, const AckermannModel& model) {
  os << "v: " << model.v() << ", theta: " << model.theta() << ", vehicleLength: " << model.vehicleLength()
     << ", vehicleWidth: " << model.vehicleWidth() << ", distanceToCenter_: " << model.distanceToCenter();
  return os;
}

/* ************************************************************************* */
void AckermannModel::print(const std::string& s) const {
  gtsam::print((Vector)vector(), s);
}

/* ************************************************************************* */
bool AckermannModel::equals(const AckermannModel& model, double tol) const {
  return (std::fabs(v_ - model.v_) < tol && std::fabs(theta_ - model.theta_) < tol &&
          std::fabs(vehicleLength_ - model.vehicleLength_) < tol && std::fabs(vehicleWidth_ - model.vehicleWidth_) < tol &&
          std::fabs(distanceToCenter_ - model.distanceToCenter_) < tol);
}

}  // namespace gtsam