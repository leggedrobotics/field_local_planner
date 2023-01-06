/**
 *  @file  GoalRmp.h
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/expressions.h>
#include <iostream>

namespace gtsam {

class GTSAM_EXPORT DifferentialModel {
 private:
  // Parameters of the model
  double theta_;

  // Auxiliary values
  Matrix32 J_;  // Transition matrix

 public:
  enum { dimension = 1 };
  typedef boost::shared_ptr<DifferentialModel> shared_ptr;  ///< shared pointer to object

  /*
   * Constructors
   */
  DifferentialModel(double theta);

  DifferentialModel(const Pose2& state);

  Vector3 apply(const Vector2& acceleration, OptionalJacobian<3, 1> Hmodel = boost::none, OptionalJacobian<3, 2> Hacc = boost::none) const;

  Matrix3 applyMetric(const Matrix2& metric) const { return J_ * metric * J_.transpose(); }

  // Elements
  inline double theta() const { return theta_; }

  /// vectorized form (column-wise)
  Vector1 vector() const {
    Vector1 v;
    v << theta_;
    return v;
  }

  Matrix32 jacobian() const { return J_; }

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const DifferentialModel& cal);

  /// print with optional string
  void print(const std::string& s = "DifferentialModel") const;

  /// Check if equal up to specified tolerance
  bool equals(const DifferentialModel& model, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline size_t dim() const { return dimension; }

  /// return DOF, dimensionality of tangent space
  static size_t Dim() { return dimension; }

  /// Given 1-dim tangent vector, create new Differential Model
  inline DifferentialModel retract(const Vector& d) const { return DifferentialModel(theta_ + d(1)); }

  /// Unretraction for the model
  Vector1 localCoordinates(const DifferentialModel& T2) const { return T2.vector() - vector(); }
};

// Traits
template <>
struct traits<DifferentialModel> : public internal::Manifold<DifferentialModel> {};

template <>
struct traits<const DifferentialModel> : public internal::Manifold<DifferentialModel> {};

}  // namespace gtsam