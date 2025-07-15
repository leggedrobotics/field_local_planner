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
 *  @file  ControlPoint.h
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/expressions.h>
#include <iostream>

/**
 * Control point2 maps a planar acceleration to a different frame
 */

namespace gtsam {

class GTSAM_EXPORT ControlPoint2 {
 private:
  Pose2 pose_;  // Pose of the control point
  Matrix23 J_;  // Full Jacobian from fixed frame to control point frame

 public:
  enum { dimension = 2 };
  typedef boost::shared_ptr<ControlPoint2> shared_ptr;  ///< shared pointer to object

  /*
   * Constructors
   */
  ControlPoint2(const Vector2& point) { ControlPoint2(Pose2(0, point)); }

  ControlPoint2(const Pose2& T_a_b) {
    // Create zero vector to obtain the position of the control point in the fixed frame and the jacobian
    J_.setZero();
    Vector2 zero = Vector2::Zero();
    pose_.transformFrom(zero, J_);
  }

  Vector2 apply(const Vector3& acceleration, OptionalJacobian<2, 2> Hmodel = boost::none, OptionalJacobian<2, 3> Hacc = boost::none) const {
    Matrix23 J;
    Vector2 zero = Vector2::Zero();
    pose_.transformFrom(zero, J);

    if (Hmodel) {
      *Hmodel = I_2x2;
    }
    if (Hacc) {
      // *Hacc << J_;
      *Hacc << J;
    }

    // Transform acceleration
    // Vector2 output = J_ * acceleration;
    Vector2 output = J * acceleration;
    // std::cout << "- control point -" << std::endl;
    // std::cout << J_ << std::endl;
    // std::cout << Hacc << std::endl;
    // std::cout << acceleration.transpose() << std::endl;
    // std::cout << "output: " << output.transpose() << std::endl;
    // std::cout << "----" << std::endl;

    return output;
  }

  Vector2 point() const { return pose_.translation(); }
  Pose2 pose() const { return pose_; }
  Matrix23 jacobian() { return J_; }

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const ControlPoint2& cal);

  /// print with optional string
  void print(const std::string& s = "ControlPoint2") const;

  /// Check if equal up to specified tolerance
  bool equals(const ControlPoint2& model, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline size_t dim() const { return dimension; }

  /// return DOF, dimensionality of tangent space
  static size_t Dim() { return dimension; }

  /// Given 5-dim tangent vector, create new Control point Model
  inline ControlPoint2 retract(const Vector& d) const { return ControlPoint2(point() + d); }

  /// Unretraction for the model
  Vector2 localCoordinates(const ControlPoint2& T2) const { return T2.point() - point(); }

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

// Traits
template <>
struct traits<ControlPoint2> : public internal::Manifold<ControlPoint2> {};

template <>
struct traits<const ControlPoint2> : public internal::Manifold<ControlPoint2> {};

}  // namespace gtsam