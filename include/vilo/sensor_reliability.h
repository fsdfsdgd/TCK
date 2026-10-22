#pragma once

#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace vilo {

// No historical comparison exists on the first fused window, or after a
// modality has had no observations. Use neutral relative reliability then.
inline double relativeSensorReliability(int count, double error,
        const std::deque<int>& counts, const std::deque<double>& errors)
{
    if(count<=0)return 0;
    if(!std::isfinite(error)||error<0)
        throw std::runtime_error("Invalid sensor residual in fused BA");
    if(counts.empty()||errors.empty())return 1;
    const double old_count=std::accumulate(counts.begin(),counts.end(),0.0)/counts.size();
    const double old_error=std::accumulate(errors.begin(),errors.end(),0.0)/errors.size();
    if(old_count<=0||old_error<=0)return 1;
    if(error==0)return std::numeric_limits<double>::infinity();
    return (double(count)/old_count)*(old_error/error);
}

inline std::pair<double,double> fusedSensorWeights(int lidar_count,int visual_count,
        double lidar_reliability,double visual_reliability,bool camera_bad)
{
    // Preserve the estimator's existing degraded-camera policy, BEFORE any
    // division by the number of visual observations.
    if(camera_bad||visual_count==0)
        return {lidar_count>0?10.0:0.0,visual_count>0?1.0:0.0};
    return {double(lidar_count)/double(visual_count)*std::atan(std::sqrt(lidar_reliability)),
            std::atan(std::sqrt(visual_reliability))};
}
} // namespace vilo
