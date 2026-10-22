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

#ifndef VILO_FRAME_H_
#define VILO_FRAME_H_

#include <sophus/se3.h>
#include <boost/noncopyable.hpp>
#include <memory>
#include <boost/thread.hpp>
#include <vilo/global.h>

#include "vilo/camera.h"
#include "vilo/imu.h"

namespace g2o {
class VertexSE3Expmap;
}
typedef g2o::VertexSE3Expmap g2oFrameSE3;

namespace vilo {

class Point;
class PhotomatricCalibration;
class IMUPriorConstraint;
struct Feature;

typedef list<Feature*> Features;
typedef vector<cv::Mat> ImgPyr;


#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class Frame : boost::noncopyable
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    IMU* imu_from_last_frame_ = nullptr; // Owned by this frame.
    IMU* imu_from_last_keyframe_ = nullptr; // Non-owning API alias.
    std::shared_ptr<IMU> key_imu_owner_;
    Eigen::Vector3d ba_, bg_;
    Eigen::Vector3d velocity_;
    Frame* last_kf_;
    Sophus::SE3 T_c_b_, T_b_c_;  
    int lba_id_;
    long align_lid_;
    bool is_opt_imu_; 
    IMUPriorConstraint* prior_constraint_; 
    double ts_;

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
    void setTimeStamp(double ts){ts_ = ts;}
    double getTimeStamp(){return ts_;}
   
    static int frame_counter_;         
    static int keyFrameCounter_;
    int id_;                    
    int keyFrameId_;
    double timestamp_; 
    string m_timestamp_s;
    vilo::AbstractCamera* cam_;  
    Sophus::SE3 T_f_w_;  
    Matrix<double, 6, 6> Cov_;                
    ImgPyr img_pyr_;           
    Features fts_;
    vector<Feature*> key_pts_; 
    bool is_keyframe_;         
    g2oFrameSE3* v_kf_;      
    int last_published_ts_;     
    ImgPyr grad_pyr_, sobelX_, sobelY_, canny_;
    float integralImage_;
    std::vector<Frame*> connectedKeyFrames;
    int lastReprojectFrameId_;
    PhotomatricCalibration* m_pc=NULL;
    ImgPyr m_pyr_raw;
    double m_exposure_time = -1;
    bool m_exposure_finish = false;
    bool m_added=false;
    bool m_kf_pc=false;

    FramePtr m_last_frame;
    float gradMean_;
    float m_error_in_px = 1.f;
    int m_n_inliers;
    void photometricallyCorrectPyramid(const cv::Mat& img_level_0, ImgPyr& pyr_correct, ImgPyr& pyr_raw, int n_levels);


    Frame(vilo::AbstractCamera* cam, const cv::Mat& img, double timestamp, PhotomatricCalibration* opc=NULL);
    Frame(vilo::AbstractCamera* cam, const cv::Mat& img, double timestamp, SE3& Tbc, PhotomatricCalibration* opc=NULL);
    ~Frame();

    void initFrame(const cv::Mat& img);

    void setKeyframe();

    void addFeature(Feature* ftr);

    void getFeaturesCopy(Features& list_copy);

    void setKeyPoints();

    void checkKeyPoints(Feature* ftr);

    void removeKeyPoint(Feature* ftr);

    inline size_t nObs() const { return fts_.size(); }

    bool isVisible(const Vector3d& xyz_w);

    inline const cv::Mat& img() const { return img_pyr_[0]; }

    inline bool isKeyframe() const { return is_keyframe_; }

    inline Vector2d w2c(const Vector3d& xyz_w) 
    { 
        boost::unique_lock<boost::mutex> lock(pose_mutex_);
        return cam_->world2cam( T_f_w_ * xyz_w );
    }

    inline Vector3d c2f(const Vector2d& px) const { return cam_->cam2world(px[0], px[1]); }

    inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

    inline Vector3d w2f(const Vector3d& xyz_w)
    { 
        boost::unique_lock<boost::mutex> lock(pose_mutex_);
        return T_f_w_ * xyz_w; 
    }

    inline Vector3d f2w(const Vector3d& f)
    { 
        boost::unique_lock<boost::mutex> lock(pose_mutex_);
        return T_f_w_.inverse() * f; 
    }

    inline Vector2d f2c(const Vector3d& f) const { return cam_->world2cam( f ); }

    inline Vector3d pos()
    { 
        boost::unique_lock<boost::mutex> lock(pose_mutex_);
        return T_f_w_.inverse().translation(); 
    }

    Sophus::SE3 getFramePose();
    void setFramePose(Sophus::SE3& Tfw);

    inline static void jacobian_xyz2uv(const Vector3d& xyz_in_f, Matrix<double,2,6>& J)
    {
        const double x = xyz_in_f[0];
        const double y = xyz_in_f[1];
        const double z_inv = 1./xyz_in_f[2];
        const double z_inv_2 = z_inv*z_inv;

        J(0,0) = -z_inv;             
        J(0,1) = 0.0;                
        J(0,2) = x*z_inv_2;          
        J(0,3) = y*J(0,2);            
        J(0,4) = -(1.0 + x*J(0,2)); 
        J(0,5) = y*z_inv;            

        J(1,0) = 0.0;                 
        J(1,1) = -z_inv;              
        J(1,2) = y*z_inv_2;          
        J(1,3) = 1.0 + y*J(1,2);      
        J(1,4) = -J(0,3);            
        J(1,5) = -x*z_inv;           
    }


    void prepareForFeatureDetect();

    void finish();

public:
    boost::mutex keypoints_mutex_;

protected:
    boost::mutex pose_mutex_;
    boost::mutex fts_mutex_;
    
};

namespace frame_utils {

    void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

    bool getSceneDepth( Frame& frame, double& depth_mean, double& depth_min);

    bool getSceneDistance( Frame& frame, double& distance_mean);

    void createImgGrad(const ImgPyr& pyr_img, ImgPyr& scharr, int n_levels);


    void createIntegralImage(const cv::Mat& image, float& integralImage);

} // namespace frame_utils
} // namespace vilo

#endif // VILO_FRAME_H_
