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

#ifndef VILO_INITIALIZATION_H
#define VILO_INITIALIZATION_H

#include <vilo/global.h>

namespace vilo {

class FrameHandlerMono;

namespace initialization {

enum InitResult { FAILURE, NO_KEYFRAME, SUCCESS };

enum class InitializerType {
    kHomography,      
    kTwoPoint,        
    kFivePoint,       
    kOneShot,         
    kStereo,          
    kArrayGeometric,  
    kArrayOptimization 
};


class KltHomographyInit {

friend class vilo::FrameHandlerMono;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FramePtr frame_ref_;
    KltHomographyInit() {};
    ~KltHomographyInit() {};
    InitResult addFirstFrame(FramePtr frame_ref);
    InitResult addSecondFrame(FramePtr frame_ref);
    void reset();

protected:
    vector<cv::Point2f> px_ref_;     
    vector<cv::Point2f> px_cur_;    

    vector<Vector3d> f_ref_;         
    vector<Vector3d> f_cur_;          

    vector<double> disparities_;      
    vector<int> inliers_;            
    vector<Vector3d> xyz_in_cur_;     
    SE3 T_cur_from_ref_;             

    InitializerType init_type_;       
    cv::Mat img_prev_;
    vector<cv::Point2f> px_prev_;
    vector<Vector3d> ftr_type_;
};


void detectFeatures(FramePtr frame, vector<cv::Point2f>& px_vec, vector<Vector3d>& f_vec, vector<Vector3d>& ftr_type);

void trackKlt(FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities,
    cv::Mat& img_prev, 
    vector<cv::Point2f>& px_prev,
    vector<Vector3d>& fts_type);


void computeInitializeMatrix(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref);

double computeP3D(
    const vector<Vector3d>& vBearingRef,
    const vector<Vector3d>& vBearingCur,
    const Matrix3d& R,
    const Vector3d& t,
    const double reproj_thresh,
    double error_multiplier2,
    vector<Vector3d>& vP3D,
    vector<int>& inliers);

bool patchCheck(
    const cv::Mat& imgPre, const cv::Mat& imgCur, const cv::Point2f& pxPre, const cv::Point2f& pxCur);

bool createPatch(const cv::Mat& img, const cv::Point2f& px, float* patch);

bool checkSSD(float* patch1, float* patch2);

Vector3d distancePointOnce(
    const Vector3d pointW, Vector3d bearingRef, Vector3d bearingCur, SE3 T_c_r);

} // namespace initialization
} // namespace vilo

#endif // VILO_INITIALIZATION_H
