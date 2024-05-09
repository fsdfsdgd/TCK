// This file is part of TCK-VILO:A Tightly-Coupled and Keyframe-Based Visual-Inertial-Lidar Odometry System for Mobile Robots With Adaptive Sensor Reliability Evaluation
//
// Copyright(c) 2024, Jun Yin
//
// This library is highly based on the previous 
// implementation of HSO(https://github.com/luodongting/HSO),   
// A-LOAM (https://github.com/HKUST-Aerial-Robotics/A-LOAM), and
// ORB-SLAM3(https://github.com/UZ-SLAMLab/ORB_SLAM3)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#ifndef VILO_FRAME_HANDLER_BASE_H_
#define VILO_FRAME_HANDLER_BASE_H_

#include <queue>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <vilo/global.h>
#include <vilo/map.h>

#include "vilo/vikit/timer.h"
#include "vilo/vikit/ringbuffer.h"

namespace vilo {
class PerformanceMonitor;}

namespace vilo
{
class Point;
class Matcher;
class DepthFilter;

class FrameHandlerBase : boost::noncopyable
{
public:
  enum Stage {
    STAGE_PAUSED,
    STAGE_FIRST_FRAME,
    STAGE_SECOND_FRAME,
    STAGE_DEFAULT_FRAME,
    STAGE_RELOCALIZING
  };
  enum TrackingQuality {
    TRACKING_INSUFFICIENT,
    TRACKING_BAD,
    TRACKING_GOOD
  };
  enum UpdateResult {
    RESULT_NO_KEYFRAME,
    RESULT_IS_KEYFRAME,
    RESULT_FAILURE
  };

  FrameHandlerBase();

  virtual ~FrameHandlerBase();

  Map* map() const { return map_; }

  void reset() { set_reset_ = true; }

  void start() { set_start_ = true; }

  Stage stage() const { return stage_; }

  TrackingQuality trackingQuality() const { return tracking_quality_; }

  double lastProcessingTime() const { return timer_.getTime(); }

  size_t lastNumObservations() const { return num_obs_last_; }

public:
  Stage stage_;                 
  bool set_reset_;          
  bool set_start_;             
  vilo::Timer timer_;           
  vilo::RingBuffer<double> acc_frame_timings_;  
  vilo::RingBuffer<size_t> acc_num_obs_;        
  size_t num_obs_last_;                        
  TrackingQuality tracking_quality_;           
  size_t regular_counter_;
  int n_acc_relocalization_;
  Map* map_;                    

  bool startFrameProcessingCommon(const double timestamp);

  int finishFrameProcessingCommon(
      const size_t update_id,
      const UpdateResult dropout,
      const size_t num_observations);

  void resetCommon();

  virtual void resetAll() { resetCommon(); }

  virtual void setTrackingQuality(const size_t num_observations);

};

} 

#endif // VILO_FRAME_HANDLER_BASE_H_
