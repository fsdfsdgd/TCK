#pragma once

#include <boost/weak_ptr.hpp>
#include <vector>
#include "vilo/lidar.h"

// Historical output owns poses and sparse keyframe features, never complete
// scans, KD trees, image pyramids or inertial optimization state.
class SlamArchive
{
public:
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int id;
        double time;
        bool key = false;
        Sophus::SE3 Twb, Twl;
        Eigen::Vector3d velocity;
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr features;
        boost::weak_ptr<vilo::Lidar> live;
    };
    using States = std::vector<State,Eigen::aligned_allocator<State>>;
    States states;

    void append(const vilo::LidarPtr& scan)
    {
        if (!states.empty() && scan->id_ <= states.back().id)
            throw std::runtime_error("LiDAR state consumed more than once or out of order");
        State state;
        state.id = scan->id_;
        state.time = scan->getTimeStamp();
        state.live = scan;
        update(state,scan);
        states.push_back(std::move(state));
    }

    void refresh()
    {
        live_states_ = 0;
        for (auto& state : states) {
            if (const auto scan = state.live.lock()) {
                update(state,scan);
                ++live_states_;
            }
        }
    }
    size_t liveStates() const { return live_states_; }

private:
    size_t live_states_ = 0;
    static void update(State& state, const vilo::LidarPtr& scan)
    {
        state.Twb = scan->getImuPose();
        state.Twl = scan->getLidarPose().inverse();
        state.velocity = scan->getVelocity();
        state.key = scan->is_keylidar_ || scan->id_ == 0;
        if (state.key) state.features = scan->ptr_less_surfs_;
        if (!state.Twb.matrix().allFinite() || !state.Twl.matrix().allFinite())
            throw std::runtime_error("Non-finite archived SLAM pose");
    }
};
