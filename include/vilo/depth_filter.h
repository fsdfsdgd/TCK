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

#ifndef VILO_DEPTH_FILTER_H_
#define VILO_DEPTH_FILTER_H_

#include <queue>
#include <map>
#include <boost/weak_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include "vilo/global.h"
#include "vilo/feature_detection.h"
#include "vilo/reprojector.h"
#include "vilo/matcher.h"
#include "vilo/IndexThreadReduce.h"
#include "vilo/bundle_adjustment.h"
#include "vilo/frame_handler_mono.h"
#include "vilo/vikit/performance_monitor.h"
#include "vilo/map.h"

namespace vilo {

class Frame;
class Feature;
class Point;
class FrameHandlerMono;
class Map;
class Lidar;

struct Seed
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int batch_counter;
    static int seed_counter;
    int batch_id;                
    int id;                      
    Feature* ftr;                
    float a;                     
    float b;                    
    float mu;                    
    float z_range;               
    float sigma2;                
    Matrix2d patch_cov;          
    vector<FramePtr> pre_frames; 

    bool isValid;

    Vector2i eplStart;
    Vector2i eplEnd;

    bool haveReprojected;
    Point* temp;

    std::vector<float> vec_distance;
    std::vector<float> vec_sigma2;

    std::vector<FramePtr> optFrames_P;
    std::vector<FramePtr> optFrames_A;
    float opt_id;

    FramePtr last_update_frame;
    Vector2d last_matched_px;
    int last_matched_level;

    float converge_thresh;

    bool is_update;

    Seed(Feature* ftr, float depth_mean, float depth_min, float converge_threshold=200);
};

class DepthFilter
{
friend class vilo::Reprojector;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::unique_lock<boost::mutex> lock_t;
  typedef boost::function<void ( Point*, double )> callback_t;

  struct Options
  {
    bool check_ftr_angle;                
    bool epi_search_1d;                        
    bool verbose;                               
    bool use_photometric_disparity_error;       
    int max_n_kfs;                            
    double sigma_i_sq;                          
    double seed_convergence_sigma2_thresh;      
    Options()
    : check_ftr_angle(false),
      epi_search_1d(false),
      verbose(true),
      use_photometric_disparity_error(false),
      max_n_kfs(3),
      sigma_i_sq(5e-4),
      seed_convergence_sigma2_thresh(200.0)
    {}
  } options_;

  boost::mutex stats_mut_;
  RunningStats* runningStats_;
  int n_update_last_;
  FramePtr active_frame_;
  boost::mutex mean_mutex_;
  size_t nMeanConvergeFrame_ = 6;

  DepthFilter(feature_detection::FeatureExtractor* featureExtractor, callback_t seed_converged_cb);
  virtual ~DepthFilter();
  void startThread();
  void stopThread();
  // Offline playback must drain mapping before reading optimized states or
  // advancing the next input; otherwise its unpaced input drops queued work.
  void waitUntilIdle();
  void addFrame(FramePtr frame);
  void addKeyframe(FramePtr frame, double depth_mean, double depth_min, float converge_thresh=200.0);
  void removeKeyframe(FramePtr frame);
  void reset();
  void getSeedsCopy(const FramePtr& frame, std::list<Seed>& seeds);
  std::list<Seed>& getSeeds() { return seeds_; }
  static void updateSeed(const float x, const float tau2, Seed* seed);

  static double computeTau(
      const SE3& T_ref_cur,
      const Vector3d& f,
      const double z,
      const double px_error_angle);


  void directPromotionFeature();

  bool is_new_lidar_ = false;
  bool is_new_kf_ = false;
  bool is_visual_opt_;
  bool is_track_good_;
  float imu_freq_, imu_na_, imu_ng_, imu_naw_, imu_ngw_;

  FramePtr last_frame_;
  set<Frame*> sub_map_;
  vector< pair<FramePtr,size_t> > overlap_kfs_;

  FrameHandlerMono* frame_handler_;
  LidarPtr new_lidar_;
  LidarPtr last_lidar_;
  LidarPtr last_keylidar_;
  std::queue<LidarPtr> lidar_queue_;
  int frameCount;
  int laserCloudCenWidth;
  int laserCloudCenHeight;
  int laserCloudCenDepth;
  int laserCloudWidth;
  int laserCloudHeight;
  int laserCloudDepth;
  int feature_array_num;

  list< LidarPtr > key_lidars_list_;

  // Active geometry is stored in keyframe coordinates, not baked into world
  // coordinates before the keyframe has finished local bundle adjustment.
  struct LidarSubmap {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct Scan {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      boost::weak_ptr<Lidar> lidar;
      Sophus::SE3 relative;
      Eigen::Vector3d velocity;
    };
    LidarPtr anchor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr points;
    std::vector<Scan,Eigen::aligned_allocator<Scan>> scans;
  };
  std::map<int,LidarSubmap> active_lidar_submaps_;
  void synchronizeLidarSubmaps();
  int lidarCubeIndex(const Eigen::Vector3d& point) const;
  void freezeLidarSubmaps();
  void addLidarSubmap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& features);
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_features_array[4851];
  pcl::PointCloud<pcl::PointXYZI>::Ptr surf_features_array[4851]; 
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_local_map_;
  Map* map_;

  void setFrameHandler(FrameHandlerMono* frame_handler);
  void processKeyFrame(bool is_tracking_good);
  void addLidar(LidarPtr lidar);
  void addKeyframe(FramePtr frame, bool is_track_good, double depth_mean, double depth_min, float converge_thresh=200.0);
  void trackLidarLocalMap();
  void liLocalMapping(list< LidarPtr >& key_lidars_list,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map,
                    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr local_map_kd_tree, 
                    bool is_opt_each_bias=true);
  void lioInitialization();
  void vioInitialization();
  void alignVioLio();
  bool getHaltFlag();
  void setHaltFlag(bool flag);
  void localMapping(bool is_tracking_good);
  void predictWithImu();
  ofstream ot_;

protected:
  boost::mutex detector_mut_;

  feature_detection::FeatureExtractor* featureExtractor_;

  callback_t seed_converged_cb_;

  std::list<Seed> seeds_;
  boost::mutex seeds_mut_;

  boost::mutex m_converge_seed_mut;
  std::list<Seed> m_converge_seed;


  bool seeds_updating_halt_;           
  boost::thread* thread_;
  std::queue<FramePtr> frame_queue_;
  boost::mutex frame_queue_mut_;
  boost::condition_variable frame_queue_cond_;
  boost::condition_variable idle_cond_;
  bool processing_frame_ = false;

  FramePtr new_keyframe_;             
  boost::mutex new_keyframe_mut_;
  bool new_keyframe_set_;               
  double new_keyframe_min_depth_;       
  double new_keyframe_mean_depth_;     
  vilo::PerformanceMonitor permon_;     
  Matcher matcher_; 

  float convergence_sigma2_thresh_ = 200.0;


  lsd_slam::IndexThreadReduce* threadReducer_;
  double px_error_angle_;
  
  void initializeSeeds(FramePtr frame);

  virtual void updateSeeds(FramePtr frame);

  void clearFrameQueue();

  void run();

  void observeDepth();
  void observeDepthRow(int yMin, int yMax, RunningStats* stats);


  void observeDepthWithPreviousFrameOnce(std::list<Seed>::iterator& ite);

  boost::mutex lidar_queue_mut_;
  boost::mutex halt_mutex_;

private:

  std::vector< list<FramePtr> > frame_prior_;

  size_t n_pre_update_, n_pre_try_;

  size_t nPonits, nSkipFrame;
  vector<size_t> m_v_n_converge;

  bool activatePoint(Seed& seed, bool& isValid);
  void seedOptimizer(Seed& seed, 
    const vector<pair<FramePtr, Vector2d> >& targets, 
    const vector<Vector2d>& normals);
  
  void setNewKeyframeFlag(bool flag);
  bool getNewKeyframeFlag();
};

} // namespace vilo

#endif // VILO_DEPTH_FILTER_H_
