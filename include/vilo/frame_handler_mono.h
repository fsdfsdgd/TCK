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

#ifndef VILO_FRAME_HANDLER_H_
#define VILO_FRAME_HANDLER_H_

#include <set>
#include <boost/thread.hpp>

#include "vilo/vikit/performance_monitor.h"
#include "vilo/frame_handler_base.h"
#include "vilo/reprojector.h"
#include "vilo/initialization.h"
#include "vilo/photomatric_calibration.h"
#include "vilo/camera.h"
#include "vilo/imu.h"
#include "vilo/coarse_tracker.h"
#include "vilo/lidar.h"
#include "vilo/config.h"
#include "vilo/map.h"
#include "vilo/frame.h"
#include "vilo/feature.h"
#include "vilo/point.h"
#include "vilo/pose_optimizer.h"
#include "vilo/matcher.h"
#include "vilo/feature_alignment.h"
#include "vilo/global.h"
#include "vilo/depth_filter.h"

namespace vilo {

class FrameHandlerMono : public FrameHandlerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  FrameHandlerMono(vilo::AbstractCamera* cam, bool _use_pc=false);
  FrameHandlerMono(std::string& settingfile, bool _use_pc=false);
  virtual ~FrameHandlerMono();

  void addImageImu(const cv::Mat& image, 
                  const std::vector<vilo::IMUPoint>& imu_vec, 
                  const double& img_time,
                  UpdateResult& res);


  void addLidarImu(pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud,
                   const std::vector<vilo::IMUPoint>& imu_vec, 
                   const double& timestamp,
                   std::vector<double>& pts_ts_vec);

  void addLidarImageImu(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar,
                        cv::Mat& image,
                        const std::vector<vilo::IMUPoint>& imu_vec,
                        std::vector<double>& pts_ts_vec, 
                        const double& lidar_ts,
                        const double& image_ts,
                        int imu_id1, int imu_id2,
                        int imu_id3, int imu_id4,
                        int type);

  const set<FramePtr>& coreKeyframes() { return core_kfs_; }

  FramePtr lastFrame() { return last_frame_; }

  LidarPtr lastLidar() { return last_lidar_; }

  const vector<cv::Point2f>& initFeatureTrackRefPx() const { return klt_homography_init_.px_ref_; }
  const vector<cv::Point2f>& initFeatureTrackCurPx() const { return klt_homography_init_.px_cur_; }

  DepthFilter* depthFilter() const { return depth_filter_; }

public:
  std::vector<double> times_;
  std::vector<Sophus::SE3> gt_poses_;
  std::list< pair<long, Sophus::SE3> > visual_motions_;
  std::list< pair<long, Sophus::SE3> > lidar_motions_;
  list< LidarPtr > key_lidars_list_; 
  Lidar* last_key_lidar_; 

  std::string setting_file_;
  vilo::IMU* imu_from_last_keyframe_;            
  vilo::IMU* imu_from_last_keylidar_; 
  bool is_imu_used_;
  bool is_vio_initialized_;
  bool is_lio_initialized_;
  bool is_livo_initialized_;
  bool is_lidar_img_pair_;
  long livo_init_lidar_id_;
  float imu_freq_, imu_na_, imu_ng_, imu_naw_, imu_ngw_;
  bool is_third_opt_ = false; 

  FramePtr last_keyframe_;
  SE3 T_b_c_;
  vilo::AbstractCamera* cam_;                    
  Reprojector* reprojector_;                    
  FramePtr new_frame_;                         
  FramePtr last_frame_;                 
  FramePtr first_frame_;
  set<FramePtr> core_kfs_;                      
 initialization::KltHomographyInit klt_homography_init_; 
  DepthFilter* depth_filter_;                
  Sophus::SE3 lidar_motion_model_, visual_motion_model_;
  bool after_init_ = false;
  PhotomatricCalibration* photomatric_calib_;
  vector<Frame*> covisible_kfs_; 
  set<Frame*> sub_map_;
  size_t n_matched_fts_, n_edges_final_; 
  double distance_mean_, depth_min_;

  LidarPtr last_lidar_;
  LidarPtr new_lidar_;
  int n_lidar_;
  Sophus::SE3 T_b_l_;
  long lio_init_id_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_flat_pts_;
  int step_ = 5;
  bool is_first_lidar_;
 
  bool readConfigFile(std::string& settingfile);
  
  void preintegrate(const std::vector<vilo::IMUPoint>& imu_vec, int type); 
  void predictWithImu();
  void trackNewLidarFrame(LidarPtr& new_lidar_, LidarPtr& last_lidar_, Sophus::SE3& Trc); 
  virtual void initialize();

  virtual UpdateResult processFirstFrame();
  virtual UpdateResult processSecondFrame();
  virtual UpdateResult processFrame();
  virtual UpdateResult relocalizeFrame(const SE3& T_cur_ref, FramePtr ref_keyframe);
  virtual void resetAll();

  virtual bool needNewKf(
  const double& scene_depth_mean, 
  const size_t& num_observations);

  static bool frameCovisibilityComparator(pair<int, Frame*>& lhs, pair<int, Frame*>& rhs);

  void createCovisibilityGraph(FramePtr currentFrame, size_t n_closest, bool is_keyframe);

  vector< pair<string, double> > m_stamp_et;
  vector<double> m_grad_mean;
  vector<Vector3d> m_vig;

};

} // namespace vilo

#endif // VILO_FRAME_HANDLER_H_
