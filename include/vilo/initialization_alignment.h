#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <sophus/se3.h>

namespace vilo {

struct InitializationAlignment {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    double scale = 0;
    double position_rmse = std::numeric_limits<double>::infinity();
    double rotation_rmse = std::numeric_limits<double>::infinity();
    bool accepted = false;
};

// Both trajectories describe the SAME IMU body in different world frames.
// Use attitude correspondences to constrain the otherwise unobservable rotation
// about a nearly straight translation trajectory. Then solve metric scale and
// translation in closed form, without an identity-seeded Sim3 local minimum.
inline InitializationAlignment alignInitializationPoses(
    const std::vector<Sophus::SE3>& visual,
    const std::vector<Sophus::SE3>& lidar)
{
    InitializationAlignment result;
    if (visual.size() != lidar.size() || visual.size() < 6) return result;
    Eigen::Matrix3d rotation_sum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d pv = Eigen::Vector3d::Zero(), pl = pv;
    for (size_t i = 0; i < visual.size(); ++i) {
        if (!visual[i].matrix().allFinite() || !lidar[i].matrix().allFinite()) return result;
        rotation_sum += lidar[i].rotation_matrix() * visual[i].rotation_matrix().transpose();
        pv += visual[i].translation();
        pl += lidar[i].translation();
    }
    const double n = visual.size();
    pv /= n; pl /= n;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d signs(1, 1, (svd.matrixU()*svd.matrixV().transpose()).determinant());
    result.rotation = svd.matrixU()*signs.asDiagonal()*svd.matrixV().transpose();
    double variance = 0, covariance = 0, lidar_variance = 0;
    for (size_t i = 0; i < visual.size(); ++i) {
        const Eigen::Vector3d v = visual[i].translation()-pv;
        const Eigen::Vector3d l = lidar[i].translation()-pl;
        variance += v.squaredNorm();
        lidar_variance += l.squaredNorm();
        covariance += l.dot(result.rotation*v);
    }
    // Scale cannot be observed from stationary positions; wait for more data.
    if (variance/n < 1e-6 || lidar_variance/n < 0.01) return result;
    result.scale = covariance/variance;
    if (!std::isfinite(result.scale) || result.scale <= 1e-6) return result;
    result.translation = pl-result.scale*result.rotation*pv;
    double position_error = 0, rotation_error = 0;
    for (size_t i = 0; i < visual.size(); ++i) {
        position_error += (lidar[i].translation()-result.scale*result.rotation*
                          visual[i].translation()-result.translation).squaredNorm();
        const Eigen::Matrix3d residual = lidar[i].rotation_matrix().transpose()*
                                        result.rotation*visual[i].rotation_matrix();
        const double angle = std::acos(std::max(-1.0, std::min(1.0, (residual.trace()-1)*0.5)));
        rotation_error += angle*angle;
    }
    result.position_rmse = std::sqrt(position_error/n);
    result.rotation_rmse = std::sqrt(rotation_error/n);
    // Stage II must not mutate either map before spatial consistency passes.
    result.accepted = std::isfinite(result.position_rmse) &&
        result.position_rmse <= 0.5 && result.rotation_rmse <= 10.0*M_PI/180.0;
    return result;
}

} // namespace vilo
