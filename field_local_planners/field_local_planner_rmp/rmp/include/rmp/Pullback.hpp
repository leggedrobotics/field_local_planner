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
 *  @file  Metric.hpp
 *  @brief Implements helper functions to make metrics
 *  @author Matias Mattamala
 **/

#pragma once

#include <rmp/models/AckermannModel.hpp>
#include <rmp/models/ControlPoint.hpp>
#include <rmp/models/DifferentialModel.hpp>

namespace gtsam {
namespace rmp {

// ControlPoint2
using ControlPoint2_ = Expression<ControlPoint2>;

template <class ACC>
inline Vector2 applyControlPoint2Helper(const ControlPoint2& control_point, const ACC& acceleration,
                                        OptionalJacobian<2, 2> Hmodel = boost::none, OptionalJacobian<2, 3> Hacc = boost::none) {
  return control_point.apply(acceleration, Hmodel, Hacc);
}

template <class ACC>
Vector2_ pullback(const Expression<ControlPoint2>& model_, const Expression<ACC>& acceleration_) {
  return Vector2_(applyControlPoint2Helper<ACC>, model_, acceleration_);
}

// AckermannModel
using AckermannModel_ = Expression<AckermannModel>;

template <class ACC>
inline Vector3 applyAckermannModelHelper(const AckermannModel& model, const ACC& acceleration, OptionalJacobian<3, 5> Hmodel = boost::none,
                                         OptionalJacobian<3, 2> Hacc = boost::none) {
  return model.apply(acceleration, Hmodel, Hacc);
}

template <class ACC>
Vector3_ pullback(const Expression<AckermannModel>& model_, const Expression<ACC>& acceleration_) {
  return Vector3_(applyAckermannModelHelper<ACC>, model_, acceleration_);
}

// DifferentialModel
using DifferentialModel_ = Expression<DifferentialModel>;

template <class ACC>
inline Vector3 applyDifferentialModelHelper(const DifferentialModel& model, const ACC& acceleration,
                                            OptionalJacobian<3, 1> Hmodel = boost::none, OptionalJacobian<3, 2> Hacc = boost::none) {
  return model.apply(acceleration, Hmodel, Hacc);
}

template <class ACC>
Vector3_ pullback(const Expression<DifferentialModel>& model_, const Expression<ACC>& acceleration_) {
  return Vector3_(applyDifferentialModelHelper<ACC>, model_, acceleration_);
}

}  // namespace rmp
}  // namespace gtsam