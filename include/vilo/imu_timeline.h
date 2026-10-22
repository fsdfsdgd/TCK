#pragma once

#include "vilo/global.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <sophus/se3.h>
#include "vilo/imu.h"

namespace vilo {

// Clip BOTH ends of each measured interval. Samples before a scan-state anchor
// are needed for deskew, but must never become negative preintegration steps.
template<class Visitor>
void forImuInterval(const std::vector<IMUPoint>& imu, double first, double last,
                    Visitor visit)
{
    if (imu.size()<2 || last<=first || first<imu.front().t || last>imu.back().t)
        throw std::runtime_error("IMU does not bracket the requested interval");
    for (size_t i=1;i<imu.size();++i) {
        const auto& a=imu[i-1]; const auto& b=imu[i];
        if (b.t<=a.t) throw std::runtime_error("Non-increasing IMU interval");
        const double lo=std::max(first,a.t),hi=std::min(last,b.t);
        if (hi<=lo) continue;
        const double alpha=((lo-a.t)+(hi-a.t))/(2*(b.t-a.t));
        const Eigen::Vector3d acc=a.acc+alpha*(b.acc-a.acc);
        const Eigen::Vector3d gyro=a.gyro+alpha*(b.gyro-a.gyro);
        visit(acc,gyro,hi-lo);
    }
}

class ImuTimeline {
public:
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double time;
        Sophus::SE3 pose;
        Eigen::Vector3d velocity;
    };
    using States=std::vector<State,Eigen::aligned_allocator<State>>;

    ImuTimeline(const std::vector<IMUPoint>& imu,double anchor,double first,double last,
                const Sophus::SE3& pose,const Eigen::Vector3d& velocity,
                const Eigen::Vector3d& ba,const Eigen::Vector3d& bg)
    {
        if (imu.size()<2 || first>anchor || last<anchor ||
            first<imu.front().t || last>imu.back().t)
            throw std::runtime_error("Full LiDAR scan is not covered by IMU samples");
        std::vector<double> times{first,anchor,last};
        for(size_t i=0;i<imu.size();++i) {
            if(i && imu[i].t<=imu[i-1].t)
                throw std::runtime_error("Non-increasing deskew IMU samples");
            if(imu[i].t>first&&imu[i].t<last)times.push_back(imu[i].t);
        }
        std::sort(times.begin(),times.end());
        times.erase(std::unique(times.begin(),times.end()),times.end());
        states_.resize(times.size());
        const size_t center=std::lower_bound(times.begin(),times.end(),anchor)-times.begin();
        states_[center]={anchor,pose,velocity};
        auto integrate=[&](const State& from,double to) {
            const double dt=to-from.time,mid=(to+from.time)*0.5;
            auto upper=std::upper_bound(imu.begin(),imu.end(),mid,
                [](double t,const IMUPoint& p){return t<p.t;});
            if(upper==imu.begin()||upper==imu.end())
                throw std::runtime_error("Missing deskew IMU segment");
            const auto& a=*(upper-1); const auto& b=*upper;
            const double alpha=(mid-a.t)/(b.t-a.t);
            const Eigen::Vector3d gyro=a.gyro+alpha*(b.gyro-a.gyro)-bg;
            const Eigen::Vector3d acc=a.acc+alpha*(b.acc-a.acc)-ba;
            const Eigen::Matrix3d R=from.pose.rotation_matrix();
            const Eigen::Matrix3d Rmid=R*Sophus::SO3::exp(gyro*(dt*0.5)).matrix();
            const Eigen::Vector3d aw=Rmid*acc+Eigen::Vector3d(0,0,-GRAVITY_VALUE);
            State out;
            out.time=to;
            out.pose=Sophus::SE3(R*Sophus::SO3::exp(gyro*dt).matrix(),
                from.pose.translation()+from.velocity*dt+0.5*aw*dt*dt);
            out.velocity=from.velocity+aw*dt;
            return out;
        };
        for(size_t i=center;i>0;--i)states_[i-1]=integrate(states_[i],times[i-1]);
        for(size_t i=center+1;i<times.size();++i)states_[i]=integrate(states_[i-1],times[i]);
    }

    Sophus::SE3 at(double time) const {
        if(!std::isfinite(time)||time<states_.front().time||time>states_.back().time)
            throw std::runtime_error("LiDAR point lies outside deskew timeline");
        auto b=std::lower_bound(states_.begin(),states_.end(),time,
            [](const State& s,double t){return s.time<t;});
        if(b==states_.begin()||b->time==time)return b->pose;
        const auto& a=*(b-1);
        const double alpha=(time-a.time)/(b->time-a.time);
        Eigen::Quaterniond qa(a.pose.rotation_matrix()),qb(b->pose.rotation_matrix());
        return Sophus::SE3(qa.slerp(alpha,qb).toRotationMatrix(),
            a.pose.translation()+alpha*(b->pose.translation()-a.pose.translation()));
    }
private:
    States states_;
};
} // namespace vilo
