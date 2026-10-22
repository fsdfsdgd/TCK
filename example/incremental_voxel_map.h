#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include "vilo/lidar.h"

// Replace a keyframe's old contribution after optimization; do not accumulate
// both its old and new world-coordinate versions. Local features stay immutable.
class IncrementalVoxelMap
{
    struct Key {
        int64_t x,y,z;
        bool operator==(const Key& b) const { return x==b.x && y==b.y && z==b.z; }
    };
    struct Hash {
        size_t operator()(const Key& k) const {
            size_t h=std::hash<int64_t>{}(k.x);
            h ^= std::hash<int64_t>{}(k.y)+0x9e3779b9+(h<<6)+(h>>2);
            return h^(std::hash<int64_t>{}(k.z)+0x9e3779b9+(h<<6)+(h>>2));
        }
    };
    struct Cell {
        Eigen::Vector3d sum=Eigen::Vector3d::Zero();
        size_t count=0;
    };
    struct Contribution {
        Sophus::SE3 pose;
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
    };
    double size_;
    std::unordered_map<Key,Cell,Hash> cells_;
    std::unordered_map<int,Contribution> frames_;
    size_t input_points_=0;

    void apply(const Contribution& frame, bool add)
    {
        for (const auto& p : frame.cloud->points) {
            Eigen::Vector3d q=frame.pose*Eigen::Vector3d(p.x,p.y,p.z);
            if (!q.allFinite()) throw std::runtime_error("Non-finite SLAM feature");
            Key key{int64_t(std::floor(q.x()/size_)),int64_t(std::floor(q.y()/size_)),
                    int64_t(std::floor(q.z()/size_))};
            if (add) {
                auto& cell=cells_[key]; cell.sum+=q; ++cell.count;
            } else {
                auto it=cells_.find(key);
                if (it==cells_.end() || it->second.count==0)
                    throw std::logic_error("Missing old keyframe voxel contribution");
                it->second.sum-=q;
                if (--it->second.count==0) cells_.erase(it);
            }
        }
    }
public:
    explicit IncrementalVoxelMap(double size):size_(size) {
        if (!std::isfinite(size) || size<=0) throw std::invalid_argument("Invalid map voxel size");
    }
    void update(int id,const Sophus::SE3& pose,
                const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
    {
        if (!cloud) return;
        auto old=frames_.find(id);
        if (old!=frames_.end()) {
            if (old->second.cloud==cloud &&
                (old->second.pose.matrix()-pose.matrix()).cwiseAbs().maxCoeff()<1e-12) return;
            apply(old->second,false);
            input_points_-=old->second.cloud->size();
        }
        Contribution frame{pose,cloud};
        apply(frame,true);
        input_points_+=cloud->size();
        frames_[id]=std::move(frame);
    }
    size_t size() const { return cells_.size(); }
    size_t inputPoints() const { return input_points_; }
    template<class Visitor> void visit(Visitor visitor) const {
        for (const auto& entry : cells_)
            visitor(entry.second.sum/double(entry.second.count));
    }
};
