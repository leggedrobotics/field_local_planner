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
#include <rmp/models/ControlPoint.hpp>

#include <cmath>
#include <fstream>
#include <iostream>

namespace gtsam {

std::ostream& operator<<(std::ostream& os, const ControlPoint2& model) {
  os << "x: " << model.point()(0) << ", y: " << model.point()(1);
  return os;
}

/* ************************************************************************* */
void ControlPoint2::print(const std::string& s) const {
  gtsam::print((Vector)point(), s);
}

/* ************************************************************************* */
bool ControlPoint2::equals(const ControlPoint2& model, double tol) const {
  return (std::fabs(point()(0) - model.point()(0)) < tol && std::fabs(point()(1) - model.point()(1)) < tol);
}

}  // namespace gtsam