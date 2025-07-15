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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>

#include "timer.hpp"

// From the Visual Teach and Repeat (VTR) package

// This class is designed to be used in a module
// Hence, it's not thread safe and aims to measure local events
// Has a single timer to track everything
// It also computes online statistics using an exponential moving average

class Profiler {
  static constexpr double BUFFER_SIZE = 10000;

 public:
  // The Event class is to keep track of the performance of an 'event'
  // in terms of computational time
  class Event {
   public:
    // Constructor
    Event(std::string name = "Unnamed", Timer::Units units = Timer::millisecs, double alpha = 0.3);
    // Destructor
    ~Event();

    // Start tracking
    void start();
    // Finish tracking
    void end();

    // Adds a new measurement to the event
    void updateStatistics(double dt);

    // Access to class members (const)
    std::string name() const;
    double lastMeasurement() const;
    double mean() const;
    double std() const;
    double max() const;
    double min() const;

    // Print nice report
    std::string getNiceReport();

    // Overload of the operator <<
    // TODO make this compliant with csv instead
    // friend std::ostream &operator<<(std::ostream &os, const Event &e);

   private:
    Timer timer_;  // Internal timer of the event

    std::string name_;  // The event name
    double lastDt_;     // The last added measurement

    double average_;  // Current computed average
    double var_;      // Current variance (squared std)
    double maxDt_;    // Maximum measurement so far
    double minDt_;    // Minimum measurement so far
    double alpha_;    // Factor for exponential moving average

    bool isRunning_;  // Flag to check if the event started

    // Logging of measurements
    std::vector<double> measurementsTimestamp_;  // All the measurements collected so far
    std::vector<double> measurements_;           // All the measurements collected so far
  };

 public:
  // Constructor
  Profiler(std::string name);

  // Enable/disable the profiler
  void setEnabled(bool enabled);

  // Add events to be tracked by the profiler
  // Events are added dynamically
  // void addEvent(std::string event, Timer::Units units = Timer::millisecs);

  // Returns the event
  const Event& getEvent(std::string event);

  // Starts to track and event
  void startEvent(std::string event, Timer::Units units = Timer::millisecs);

  // Finishes tracking and updates the statistics of the event
  void endEvent(std::string event);

  // Returns a human-readable report of the tracked events
  std::string getReport();

 private:
  // Name of the profiler
  std::string name_;

  // The events are stored in a map, where the name of the events are the keys
  // It is not as efficient as an enum, but allows more flexibility for now
  std::map<std::string, Event> events_;

  // Flag to disable the profiler internally
  bool enabled_;

  // Mutex to make it thread-safe
  std::mutex mutex_;
};
