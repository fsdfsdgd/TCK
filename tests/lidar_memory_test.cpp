#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "vilo/lidar.h"

int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    std::cout << "Eigen alignment=" << EIGEN_DEFAULT_ALIGN_BYTES
              << " malloc_aligned=" << EIGEN_MALLOC_ALREADY_ALIGNED << std::endl;
    rosbag::Bag bag(argv[1], rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{"/os1_cloud_node/points"}));
    int scans = 0;
    for (const auto& item : view) {
        const auto msg = item.instantiate<sensor_msgs::PointCloud2>();
        if (!msg) continue;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"), y(*msg,"y"), z(*msg,"z");
        for (; x != x.end(); ++x,++y,++z) {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
            pcl::PointXYZI p; p.x=*x; p.y=*y; p.z=*z; p.intensity=0;
            cloud->push_back(p);
        }
        vilo::Lidar lidar(cloud);
        lidar.registerPoints();
        const auto features = lidar.extractFeatures();
        if (!features || lidar.kdtree_corner_ == lidar.kdtree_surf_) return 1;
        std::cout << "scan=" << scans << " surfaces=" << features
                  << " less_flat=" << lidar.ptr_less_surfs_->size() << std::endl;
        if (++scans == 3) break;
    }
    return scans == 3 ? 0 : 1;
}
