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

#ifndef VILO_MAP_H_
#define VILO_MAP_H_

#include <queue>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>
#include <vilo/global.h>

namespace vilo {

class Point;
class Feature;
class Seed;

class MapPointCandidates
{
public:
  typedef pair<Point*, Feature*> PointCandidate;
  typedef list<PointCandidate> PointCandidateList;

  boost::mutex mut_;
  boost::mutex candi_mutex_;

  PointCandidateList candidates_;
  list< Point* > trash_points_;
  list< pair<Point*, Feature*> > temporaryPoints_;

  MapPointCandidates();
  ~MapPointCandidates();

  void newCandidatePoint(Point* point, double depth_sigma2);

  void addPauseSeedPoint(Point* point);

  void addCandidatePointToFrame(FramePtr frame);

  bool deleteCandidatePoint(Point* point);

  void removeFrameCandidates(FramePtr frame);

  void reset();

  void deleteCandidate(PointCandidate& c);

  void emptyTrash();


  void changeCandidatePosition(Frame* frame);
};


class Map : boost::noncopyable
{
public:
  list< FramePtr > keyframes_;          
  list< Point* > trash_points_;        
  MapPointCandidates point_candidates_;

  long init_id_ = 0;
  list< LidarPtr > lidars_list_;
  bool is_map_changed_;
  double last_avr_li_err_;
  double last_avr_lidar_err_;
  double last_avr_visual_err_;
  double last_avr_vi_err_;
  int n_lidar_err_, n_visual_err_; 

  double avr_err_pt_;
  double avr_err_ls_;
  double avr_err_lr_;
  int v_err_num_, l_err_num_; 
  std::deque<double> avr_err_pt_deque_, avr_err_lr_deque_;
  std::deque<int> v_err_num_deque_, l_err_num_deque_;
  bool is_camera_very_bad_ = false, is_lidar_very_bad_ = false;
  int n_v_recovery_ = 0, n_l_recovery_ = 0; 
  
  boost::mutex map_mutex_; 
  
  std::map<long, Sophus::SE3> lid_poses_map_;
protected:
  boost::mutex frame_mutex_; 
  boost::mutex point_mutex_; 
  boost::mutex flag_mutex_; 
  

public:
  Map();
  ~Map();
  void scaleRotate(const Eigen::Matrix3d& Rwg, const double& scale, const Eigen::Vector3d& t);

  void scaleRotate(Sophus::SE3& Tbc, Eigen::Matrix3d& Rcl_orient, Eigen::Matrix3d& Rlc_position, Eigen::Vector3d& tlc_position, double& scale);

  void rotateLidar(const Eigen::Matrix3d& Rgw, const Eigen::Vector3d& t);

  void reset();

  void safeDeletePoint(Point* pt);

  void deletePoint(Point* pt);

  bool safeDeleteFrame(FramePtr frame);

  bool safeDeleteFrameID(int id);

  void removePtFrameRef(Frame* frame, Feature* ftr);

  void addKeyframe(FramePtr new_keyframe);

  void getCloseKeyframes(const FramePtr& frame, list< pair<FramePtr,double> >& close_kfs);

  FramePtr getClosestKeyframe(const FramePtr& frame); 

  FramePtr getFurthestKeyframe(const Vector3d& pos); 

  bool getKeyframeById(const int id, FramePtr& frame); 

  void emptyTrash();

  inline FramePtr lastKeyframe() { return keyframes_.back(); }

  inline size_t size() const { return keyframes_.size(); }

  void safeDeleteTempPoint(pair<Point*, Feature*>& p);

  bool getMapChangedFlag();
  void setMapChangedFlag(bool flag);

  void addNewLidar(LidarPtr lidar);

};

namespace map_debug {

void mapStatistics(Map* map);
void mapValidation(Map* map, int id);
void frameValidation(Frame* frame, int id);
void pointValidation(Point* point, int id);

} // namespace map_debug
} // namespace vilo

#endif // VILO_MAP_H_
