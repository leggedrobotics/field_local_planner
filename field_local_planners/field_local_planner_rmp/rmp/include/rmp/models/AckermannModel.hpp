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
 *  @file  AckermannModel.h
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/expressions.h>
#include <iostream>

/** Ackermann Model
 *  X. Meng, N. Ratliff, Y. Xiang, and D. Fox,
 * “Neural autonomous navigation with riemannian motion policy,”
 * Proc. - IEEE Int. Conf. Robot. Autom., vol. 2019-May, pp. 8860–8866, 2019.
 */

namespace gtsam {

class GTSAM_EXPORT AckermannModel {
 private:
  // Parameters of the model
  double v_;
  double theta_;
  double vehicleLength_;
  double vehicleWidth_;
  double distanceToCenter_;

  // Auxiliary values
  Matrix32 J_;             // Transition matrix
  Vector3 nonlinearTerm_;  // Non linear term

 public:
  enum { dimension = 5 };
  typedef boost::shared_ptr<AckermannModel> shared_ptr;  ///< shared pointer to object

  /*
   * Constructors
   */
  AckermannModel(double v, double theta, double vehicleLength, double vehicleWidth, double distanceToVehicle);

  AckermannModel(const Pose2& state, const Point3& velocity, double vehicleLength, double vehicleWidth, double distanceToVehicle);

  void precomputeModel();

  Vector3 apply(const Vector2& acceleration, OptionalJacobian<3, 5> Hmodel = boost::none, OptionalJacobian<3, 2> Hacc = boost::none) const;

  // Elements
  inline double v() const { return v_; }
  inline double theta() const { return theta_; }
  inline double vehicleLength() const { return vehicleLength_; }
  inline double vehicleWidth() const { return vehicleWidth_; }
  inline double distanceToCenter() const { return distanceToCenter_; }

  /// vectorized form (column-wise)
  Vector5 vector() const {
    Vector5 v;
    v << v_, theta_, vehicleLength_, vehicleWidth_, distanceToCenter_;
    return v;
  }

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const AckermannModel& cal);

  /// print with optional string
  void print(const std::string& s = "AckermannModel") const;

  /// Check if equal up to specified tolerance
  bool equals(const AckermannModel& model, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline size_t dim() const { return dimension; }

  /// return DOF, dimensionality of tangent space
  static size_t Dim() { return dimension; }

  /// Given 5-dim tangent vector, create new Ackermann Model
  inline AckermannModel retract(const Vector& d) const {
    return AckermannModel(v_ + d(0), theta_ + d(1), vehicleLength_ + d(2), vehicleWidth_ + d(3), distanceToCenter_ + d(4));
  }

  /// Unretraction for the model
  Vector5 localCoordinates(const AckermannModel& T2) const { return T2.vector() - vector(); }
};

// Traits
template <>
struct traits<AckermannModel> : public internal::Manifold<AckermannModel> {};

template <>
struct traits<const AckermannModel> : public internal::Manifold<AckermannModel> {};

}  // namespace gtsam