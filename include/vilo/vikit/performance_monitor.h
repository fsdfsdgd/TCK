// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <map>
#include <string>
#include <iostream>
#include <fstream>

#include "vilo/vikit/timer.h"

namespace vilo
{

struct LogItem
{
  double data;
  bool   set;
};

class PerformanceMonitor
{
public:
  PerformanceMonitor();
  ~PerformanceMonitor();
  void init(const std::string& trace_name, const std::string& trace_dir);
  void addTimer(const std::string& name);
  void addLog(const std::string& name);
  void writeToFile();
  void startTimer(const std::string& name);
  void stopTimer(const std::string& name);
  double getTime(const std::string& name) const;
  void log(const std::string& name, double data);

private:
  std::map<std::string, Timer>      timers_;
  std::map<std::string, LogItem>    logs_;
  std::string                       trace_name_;        //<! name of the thread that started the performance monitor
  std::string                       trace_dir_;         //<! directory where the logfiles are saved
  std::ofstream                     ofs_;

  void trace();
  void traceHeader();
};

} // namespace vk
