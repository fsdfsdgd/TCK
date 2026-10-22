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

#ifndef LMVO_LASER_POINTS_H_
#define LMVO_LASER_POINTS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <boost/noncopyable.hpp>
#include <memory>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sophus/se3.h>

#include "vilo/camera.h"
#include "vilo/imu.h"

namespace vilo {

class IMUPriorConstraint;
typedef pcl::PointXYZI PcsType;
class Lidar: boost::noncopyable{


public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
   static int frame_counter_;         
   static int keyFrameCounter_;

   int id_;
   int lidar_type_;
   int w_;
   int h_;
   float z_max_;
   float z_min_;
   double ts_;
   
   size_t N_;
   int labelCount_;

   float kw_,kh_;
   vilo::AbstractCamera* cam_;
   Eigen::Matrix<double,4,4> extrinsic_;
   Eigen::Matrix<double,3,4> velo_to_img_;
   pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud_;
   cv::Mat xy_mapimg_;
   cv::Mat label_mat_;
   bool is_vertical_;
   cv::Mat oringin_lidar_img_map_;
   
   std::vector<bool> grid_occupancy_;
   int grid_w_num_, grid_h_num_; 
   int w_grid_size_, h_grid_size_;

   int Horizon_SCAN_;
   int N_SCAN_;
   float Vertical_Range_;
   float Vertical_Bottom_;
   float Scan_Period_;
   float ang_res_x_;
   float ang_res_y_;

   bool is_undist_pts_;
   bool input_ring_time_ = false; // intensity carries hardware ring + seconds.
   cv::Mat project_img_;
   cv::Mat id_img_;
   std::vector<int> scan_sid_vec_;
   std::vector<int> scan_eid_vec_;
   std::vector<float> diff_vec_;
   pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_corners_;
   pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_surfs_;
   pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_less_corners_;
   pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_less_surfs_;
   pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_corner_;
   pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_;
   Lidar* previous_lidar_;

   IMU* imu_from_last_lidar_ = nullptr; // Owned by this scan.
   IMUPriorConstraint* prior_constraint_;
   
   IMU* imu_from_last_keylidar_ = nullptr; // Non-owning API alias.
   std::shared_ptr<IMU> key_imu_owner_;
   Lidar* last_key_lidar_; 
   bool is_keylidar_;
   
   Sophus::SE3 T_b_l_, T_l_b_; 
   Sophus::SE3 T_f_w_;
   boost::mutex pose_mutex_;
   
   Eigen::Vector3d ba_, bg_;
   Eigen::Vector3d velocity_;
   int lba_id_;
   long align_vid_;
   bool is_opt_imu_; 

   Sophus::SE3 getLidarPose();    
   void setLidarPose(Sophus::SE3& Tfw);
   Eigen::Vector3d getImuAccBias();
   Eigen::Vector3d getImuGyroBias();
   void setImuAccBias(const Eigen::Vector3d& ba);
   void setImuGyroBias(const Eigen::Vector3d& bg);
   void setVelocity(const Eigen::Vector3d& vel);
   void setVelocity();
   Eigen::Vector3d getVelocity();
   Eigen::Matrix3d getImuRotation();
   Eigen::Vector3d getImuPosition();  
   Sophus::SE3 getImuPose();  
   void setNewBias(Eigen::Vector3d& ba, Eigen::Vector3d& bg);
   void setTimeStamp(double ts) {ts_ = ts;};
   double getTimeStamp() {return ts_;};  

   int pattern_25_[25][2] = {
		             {0,0},  {-1,0},  {1,0},   {0,-1},  {0,1}, 
                             {-1,1}, {-1,-1}, {1,1},   {1,-1},  {-2,0},  
                             {2,0},  {0,-2},  {0,2},   {-2,1},  {-2,-1},
                             {2,1},  {2,-1},  {-1,2},  {-1,-2}, {1,2},
                             {1,-2}, {-2,2},  {-2,-2}, {2,2},   {2,-2}
	                   };
 
   Lidar();
   Lidar(vilo::AbstractCamera* cam, int w, int h, pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud );
   Lidar(pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud, bool input_ring_time=false);
   ~Lidar();

   void projectLaserToImage(const Eigen::Matrix<double,3,4>& extrinsic,const cv::Mat& image);
   void labelground( );
   
   void setCamParams(vilo::AbstractCamera* cam);
   void setLidarParams();
   void setTbl(Sophus::SE3& Tbl){T_b_l_ = Tbl; T_l_b_ = Tbl.inverse();};
   void registerPoints();
   void undistPoints(const std::vector<IMUPoint>& imu, const std::vector<double>& pts_ts_vec);
   size_t extractFeatures();
   bool comp (int i,int j) {return (diff_vec_[i] < diff_vec_[j]);}
   void transformToStart(pcl::PointXYZI& pi, pcl::PointXYZI& po, float SCAN_PERIOD, 
                             Eigen::Quaterniond& q_last_curr, Eigen::Vector3d& t_last_curr);
   void releaseMemory(int flag=0);

   int findFeatureDepth(std::vector<cv::Point2f>& pixels , 
                          std::vector<bool>& is_depth_found,
                           std::vector<Eigen::Vector3d>& intersections);

   bool calculatePlaneCorners(const std::vector<Eigen::Matrix<double,5,1> >& label_xy_pos, 
                               const Eigen::Vector2d& px,             
                                 Eigen::Matrix<double,5,1>& corner1,
                                  Eigen::Matrix<double,5,1>& corner2,
                                   Eigen::Matrix<double,5,1>& corner3);

   bool findBestNbrs(std::vector<Eigen::Matrix<double,5,1> >& nbrs,
                      std::vector<Eigen::Matrix<double,5,1> >& best_nbrs,
                       int px_label);

   bool checkPlanar(const Eigen::Vector3d& corner1,
                     const Eigen::Vector3d& corner2,
                       const Eigen::Vector3d& corner3);

   bool isPointInTriangle(const Eigen::Vector2d& A, 
                           const Eigen::Vector2d& B, 
                            const Eigen::Vector2d& C, 
                             const Eigen::Vector2d& P);
   bool pca(std::vector<Eigen::Matrix<double,5,1> >& best_nbrs , Eigen::Vector3d& pts_norm);

   bool findDepthOnce(Eigen::Vector2d pixel , Eigen::Vector3d& intersection);

   void findGroundFeatureDepth(std::vector<Eigen::Vector2d>& pixels , 
                                std::vector<int>& index_in_pxs,
                                 std::vector<Eigen::Vector3d>& intersections);
   
   void finish();

   bool setGridOccpuancy(const std::vector<Eigen::Vector2d>& pxs);
   void resetGrid();
   void chooseExtraLaserFeatrues(std::vector<Eigen::Vector3d>& features);

   

};


} // namespace vilo

#endif // VILO_LASER_POINTS_H_
