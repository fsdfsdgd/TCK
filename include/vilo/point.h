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

#ifndef VILO_POINT_H_
#define VILO_POINT_H_

#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>
#include <vilo/global.h>

#include "vilo/vikit/math_utils.h"

namespace g2o {
class VertexSBAPointXYZ; }
typedef g2o::VertexSBAPointXYZ g2oPoint;



namespace vilo {

class Feature;
class VertexSBAPointID;

typedef Matrix<double, 2, 3> Matrix23d;

class Point : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum PointType {TYPE_DELETED, TYPE_TEMPORARY, TYPE_CANDIDATE, TYPE_UNKNOWN, TYPE_GOOD};
    enum FeatureType {FEATURE_GRADIENT, FEATURE_EDGELET, FEATURE_CORNER};

    bool is_imu_rectified;
    int lba_id_;
    static int point_counter_;
    int id_;           
    Vector3d pos_;
    Vector3d normal_;           
    Matrix3d normal_information_;   
    bool normal_set_;
    list<Feature*> obs_;                    
    size_t n_obs_;
    g2oPoint* v_pt_;  
    int last_published_ts_;
    int last_projected_kf_id_;
    PointType type_;
    int n_failed_reproj_;
    int n_succeeded_reproj_; 
    int last_structure_optim_; 
    FeatureType ftr_type_; 
    int seedStates_;
    bool isBad_;  
    double idist_;
    Feature* hostFeature_;
    VertexSBAPointID* vPoint_;
    size_t nBA_;

    //Viewer
    float color_ = 128;

    Feature* m_last_feature = NULL;
    int m_last_feature_kf_id = -1;
    vector<double> m_rad_est;

    bool isLaserDepth_;
    int latestDepthFrameId_;
    Vector3d latestLidarPos_;    

public:     
    boost::mutex obs_mutex_;

protected:
    boost::mutex point_mutex_; 

public:
    Point(const Vector3d& pos);
    Point(const Vector3d& pos, Feature* ftr);
    ~Point();

    void addFrameRef(Feature* ftr);
    int getObsSize();
    std::list<Feature*> getObs();
    bool deleteFrameRef(Frame* frame);
    void initNormal();
    Feature* findFrameRef(Frame* frame);
    bool getCloseViewObs(const Vector3d& pos, Feature*& obs);
    inline size_t nRefs() const { return obs_.size(); }
    double getPointIdist(); 
    Eigen::Vector3d getPointPose();
    void setPointIdistAndPose(double idist); 
    void setPointIdist(double idist);
    void setPointPose(Vector3d& pos); 
    void setPointState(int state);
    int getPointState();

    /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
    inline static void jacobian_xyz2uv(
        const Vector3d& p_in_f,const Matrix3d& R_f_w, Matrix23d& point_jac)
    {
        const double z_inv = 1.0/p_in_f[2];
        const double z_inv_sq = z_inv*z_inv;
        point_jac(0, 0) = z_inv;
        point_jac(0, 1) = 0.0;
        point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
        point_jac(1, 0) = 0.0;
        point_jac(1, 1) = z_inv;
        point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
        point_jac = - point_jac * R_f_w;
    }


    inline static void jacobian_id2uv(
        const Vector3d& p_in_f, const SE3& Tth, const double idH, const Vector3d& fH, Vector2d& point_jac)
    {
        Vector2d proj = vilo::project2d(p_in_f);
        Vector3d t_th = Tth.translation();
        Matrix3d R_th = Tth.rotation_matrix();
        Vector3d Rf = R_th*fH;

        point_jac[0] = -(t_th[0] - proj[0]*t_th[2]) / (Rf[2] + t_th[2]*idH);
        point_jac[1] = -(t_th[1] - proj[1]*t_th[2]) / (Rf[2] + t_th[2]*idH);
    }
};

} // namespace vilo

#endif // VILO_POINT_H_
