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
/**
 *  @file  RiemannianMotionPolicy.h
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <iostream>

namespace gtsam {
namespace rmp {

template <typename ACC, typename METRIC>
class RiemannianMotionPolicy {
 protected:
  using Acceleration = ACC;
  using Metric = METRIC;

  // Internal variables
  Acceleration acceleration_;
  Metric metric_;
  double weight_;
  std::string name_;
  Vector3 color_;

 public:
  // Constructors
  RiemannianMotionPolicy()
      : acceleration_(Acceleration::Zero()), metric_(Metric::Identity()), weight_(1.0), name_("rmp"), color_(1.0, 1.0, 1.0) {}

  RiemannianMotionPolicy(const Acceleration& acc, const Metric& metric, const double& weight = 1.0, const std::string& name = "rmp",
                         const Vector3& color = Vector3::Ones())
      : acceleration_(acc), metric_(metric), weight_(weight), name_(name), color_(color) {}

  // Getters
  const Acceleration acceleration() const { return acceleration_; }
  const Metric metric() const { return metric_; }
  const double weight() const { return weight_; }
  const std::string name() const { return name_; }
  const Vector3 color() const { return color_; }

  friend std::ostream& operator<<(std::ostream& os, const RiemannianMotionPolicy<ACC, Metric>& rmp) {
    os << "RMP [" << rmp.name() << "]\n";
    os << "  weight: " << rmp.weight() << "\n";
    os << "  acc:    " << rmp.acceleration().transpose() << "\n";
    os << "  metric: " << rmp.metric().diagonal().transpose() << "\n";
    os << "  color:  " << rmp.color().transpose() << "\n";
    return os;
  }
};

using Rmp1 = RiemannianMotionPolicy<Vector1, Matrix1>;
using Rmp2 = RiemannianMotionPolicy<Vector2, Matrix2>;
using Rmp3 = RiemannianMotionPolicy<Vector3, Matrix3>;

}  // namespace rmp
}  // namespace gtsam