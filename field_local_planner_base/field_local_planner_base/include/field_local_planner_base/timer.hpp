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

#include <chrono>
#include <string>

// From the Visual Teach and Repeat (VTR) package

class Timer {
 public:
  using Clock = std::chrono::high_resolution_clock;
  using Seconds = std::chrono::duration<double, std::ratio<1> >;
  using Millisecs = std::chrono::duration<double, std::milli>;
  using Microsecs = std::chrono::duration<double, std::micro>;

  enum Units { secs, millisecs, microsecs, no_preference };

  // Constructor
  Timer(Units units = Units::secs);

  // Set units
  void setUnits(Units units);
  // Get units for printing
  std::string getReadableUnits() const;

  // Reset timer
  void reset();

  // Get timestamp
  static double now();

  // Return elapsed time since the timer was created/reset
  double elapsed(Units units = Units::no_preference) const;

  // Return the time elapsed and resets the timer
  double lap(Units units = Units::no_preference);

 private:
  // This variable allocates the time when it was created/reset
  std::chrono::time_point<Clock> timer_;
  // Time unit
  Units units_;
};
