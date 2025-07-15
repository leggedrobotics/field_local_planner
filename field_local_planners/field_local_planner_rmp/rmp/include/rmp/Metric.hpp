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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <boost/optional.hpp>
#include <iostream>

namespace gtsam {
namespace rmp {

template <typename METRIC = Matrix22, typename ACC = Vector2>
class Metric {
  static constexpr double TOL = 0.01;

 public:
  // Interface to make metrics
  static METRIC make(const std::string& type,        // Type of the metric
                     const double& offset = 0.0,     // offset used for logistic-like metrics
                     const double& steepness = 0.0,  // stepness for logistic-like metrics
                     const double& state = 0.0,      // 'state' - a value to evaluate the metric
                     const ACC& acc = ACC::Zero()    // RMP acceleration (if required)
  ) {
    std::string metric_type, activation_type;
    parseType(type, metric_type, activation_type);
    return activation(activation_type, state, offset, steepness) * metric(metric_type, acc);
  }

  // Base metrics
  static METRIC metric(const std::string& type, const ACC& dir) {
    METRIC m;
    if (type == "projector") {
      m = makeProjectorMetric(dir);
    } else if (type == "orthoprojector") {
      m = makeOrthogonalProjectorMetric(dir);
    } else {
      m = makeConstantMetric();
    }
    return m;
  }

  static METRIC makeProjectorMetric(const ACC& dir) {
    return METRIC(dir * dir.transpose());
  }

  static METRIC makeOrthogonalProjectorMetric(const ACC& dir) {
    return (METRIC::Identity() - METRIC(dir * dir.transpose()));
  }

  static METRIC makeConstantMetric() { return METRIC::Identity(); }

  // Activations
  static double activation(std::string& type, double x, double x0, double k) {
    double a;

    if (type == "logistic") {
      a = logistic(x, x0, k);
    } else if (type == "invlogistic") {
      a = inverseLogistic(x, x0, k);
    } else {
      return 1.0;
    }
    return a < TOL ? TOL : a;
  }

  static double logistic(double x, double x0, double k) { return 1.0 / (1 + exp(-k * (x - x0))); }
  static double inverseLogistic(double x, double x0, double k) { return 1.0 / (1 + exp(k * (x - x0))); }

  // Helpers
  static void parseType(const std::string& type, std::string& base, std::string& activation) {
    size_t delimiter_pos = type.find(std::string("_"));

    if (delimiter_pos != std::string::npos) {
      base = type.substr(0, delimiter_pos);
      activation = type.substr(delimiter_pos + 1, type.length());
    } else {
      base = type;
      activation = type;
    }
  }
};

// Forward
using Metric2 = Metric<Matrix22, Vector2>;
using Metric3 = Metric<Matrix33, Vector3>;

}  // namespace rmp
}  // namespace gtsam