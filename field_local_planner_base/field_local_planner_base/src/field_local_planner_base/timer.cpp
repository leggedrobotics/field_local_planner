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
#include <field_local_planner_base/timer.hpp>
#include <stdexcept>

Timer::Timer(Units units) : timer_(Clock::now()), units_(units) {}

void Timer::setUnits(Timer::Units units) {
  units_ = units;
}

std::string Timer::getReadableUnits() const {
  switch (units_) {
    case Timer::Units::secs:
      return "s";
    case Timer::Units::millisecs:
      return "ms";
    case Timer::Units::microsecs:
      return "us";
    case Timer::Units::no_preference:
      return "not specified";
  }
}

void Timer::reset() {
  timer_ = Clock::now();
}

double Timer::now() {
  // From https://stackoverflow.com/a/55021025/3570362
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() * 1e-6;
}

double Timer::elapsed(Timer::Units units) const {
  Seconds dt = std::chrono::duration_cast<Seconds>(Clock::now() - timer_);

  Units requested = units;
  // If we are required to use default units, use the value saved from internal parameters
  if (units == Timer::Units::no_preference) requested = units_;

  // Compute the corresponding time using the chosen unit
  switch (requested) {
    case Timer::Units::secs:
      return dt.count();
    case Timer::Units::millisecs:
      return std::chrono::duration_cast<Millisecs>(dt).count();
    case Timer::Units::microsecs:
      return std::chrono::duration_cast<Microsecs>(dt).count();
    case Timer::Units::no_preference:
      throw std::runtime_error("Timer::elapsed cannot handle 'no_preference'");
  }
}

double Timer::lap(Timer::Units units) {
  Seconds dt = std::chrono::duration_cast<Seconds>(Clock::now() - timer_);

  Units requested = units;
  // If we are required to use default units, use the value saved from internal parameters
  if (units == Timer::Units::no_preference) requested = units_;

  // Compute the corresponding time using the chosen unit
  double elapsed;
  switch (requested) {
    case Units::secs:
      elapsed = dt.count();
      break;
    case Units::millisecs:
      elapsed = std::chrono::duration_cast<Millisecs>(dt).count();
      break;
    case Units::microsecs:
      elapsed = std::chrono::duration_cast<Microsecs>(dt).count();
      break;
    case Units::no_preference:
      throw std::runtime_error("Timer::elapsed cannot handle 'no_preference'");
  }

  timer_ = Clock::now();
  return elapsed;
}