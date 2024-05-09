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

#ifndef VILO_FEATURE_H_
#define VILO_FEATURE_H_

#include <vilo/frame.h>

namespace vilo {

struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType {CORNER, EDGELET, GRADIENT};

  FeatureType type;    
  Frame* frame;         
  Vector2d px;          
  Vector3d f;          
  int level;           
  Point* point;       
  Vector2d grad;      

  vector<double> outputs;
  vector<double> radiances;
  vector<double> outputs_grad;
  vector<double> rad_mean;
  Feature* m_prev_feature = NULL;  
  Feature* m_next_feature = NULL;
  bool m_added = false; 
  bool m_non_point = false;
  Matrix2d rotate_plane;

  Feature(Frame* _frame, const Vector2d& _px, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {
  }

  Feature(Frame* _frame, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {
  }

  Feature(Frame* _frame, Point* _point, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(_point),
    grad(1.0,0.0)
  {
  }

  Feature(Frame* _frame, const Vector2d& _px, const Vector2d& _grad, int _level) :
    type(EDGELET),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)),
    level(_level),
    point(NULL),
    grad(_grad)
  {
  }

  Feature(Frame* _frame, Point* _point, const Vector2d& _px, const Vector3d& _f, const Vector2d& _grad, int _level) :
    type(EDGELET),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(_point),
    grad(_grad)
  {
  }

  Feature(Frame* _frame, const Vector2d& _px, int _level, FeatureType _type) :
    type(_type),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {
  }

  Feature(Frame* _frame, Point* _point, const Vector2d& _px, int _level, FeatureType _type) :
    type(_type),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)),
    level(_level),
    point(_point),
    grad(1.0,0.0)
  {
  }

};

} // namespace vilo

#endif // VILO_FEATURE_H_
