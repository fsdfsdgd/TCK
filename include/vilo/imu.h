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

#ifndef VILO_IMU_H_
#define VILO_IMU_H_

#include <boost/noncopyable.hpp>
#include <Eigen/Core>
#include "vilo/camera.h"

namespace vilo {

const double GRAVITY_VALUE=9.81;

struct IMUPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Vector3d ba; 
    Eigen::Vector3d bg;

    double t;
    
    IMUPoint(const Eigen::Vector3d& _acc, const Eigen::Vector3d& _gyro,
    const Eigen::Vector3d& _ba,const Eigen::Vector3d& _bg, double _t) :
    acc(_acc),
    gyro(_gyro),
    ba(_ba),
    bg(_bg),
    t(_t){}

    IMUPoint(const Eigen::Vector3d& _acc, const Eigen::Vector3d& _gyro, double _t) :
    acc(_acc),
    gyro(_gyro),
    t(_t)
    {
      ba<<0.0, 0.0, 0.0;
      bg<<0.0, 0.0, 0.0;
    }

};


class IMU
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
    std::vector<double> time_vec_; 
    std::vector<double> dt_vec_;
    std::vector<Eigen::Vector3d> dp_vec_, dv_vec_;
    std::vector<Eigen::Matrix3d> dq_vec_;
    std::vector<Eigen::Vector3d> acc_vec_;
    std::vector<Eigen::Vector3d> gyro_vec_;
    
    double delta_t_;
    Eigen::Vector3d delta_p_, delta_v_ ;
    Eigen::Matrix3d delta_q_;
    Eigen::Vector3d ba_, bg_;
    Eigen::Vector3d new_ba_, new_bg_;
    Eigen::Matrix<double,6,1> db_;

    Eigen::Matrix<double,15,15> cov_mat_;
    Eigen::Matrix<double,6,6> noise_, bias_walk_;
    Eigen::Matrix<double,3,3> J_p_ba_, J_p_bg_, J_q_bg_, J_v_bg_, J_v_ba_;
    
    IMU(float& freq, 
        float& na, 
        float& ng, 
        float& naw, 
        float& ngw);

    IMU(float& freq, 
        float& na, 
        float& ng, 
        float& naw, 
        float& ngw,
        Eigen::Vector3d ba, 
        Eigen::Vector3d bg);

    ~IMU();
   
    void addImuPoint(const Eigen::Vector3d& acc_measurement, const Eigen::Vector3d& gyro_measurement,
                      const Eigen::Vector3d& ba, const Eigen::Vector3d& bg, const double& dt);

    void integrateDeltaR(Eigen::Matrix3d& dR, Eigen::Matrix3d& rightJac, 
               const Eigen::Vector3d& gyro, const Eigen::Vector3d& bg, double t);
    
    Eigen::Matrix3d getDeltaRotation(const Eigen::Vector3d& bg);
    Eigen::Vector3d getDeltaVelocity(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg);
    Eigen::Vector3d getDeltaPosition(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg);

    Eigen::Matrix3d getUpdateDeltaRotation();
    Eigen::Vector3d getUpdateDeltaPosition();
    Eigen::Vector3d getUpdateDeltaVelocity();

    Eigen::Matrix<double,6,1> getDeltaBias(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg);
    Eigen::Matrix<double,6,1> getDeltaBias();
    void setNewBias(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg);
    Eigen::Vector3d getAccBias(){return ba_;}
    Eigen::Vector3d getGyroBias(){return bg_;}
    void reintegrate();

    void reset();
};


} // namespace vilo

#endif // VILO_IMU_H_
