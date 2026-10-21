#pragma once

#include <map>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl/filters/voxel_grid.h>
#include "vilo/frame_handler_mono.h"

// Called only after DepthFilter::waitUntilIdle(). The map is reconstructed from
// optimized SLAM keyframe features; no raw input cloud is published here.
class RosBagOutput
{
public:
    RosBagOutput(const std::string& world, const std::string& body,
                 const Sophus::SE3& Tbc, const Sophus::SE3& Tbl,
                 const std::string& image_topic, const std::string& right_image_topic,
                 double map_voxel_size, double map_publish_period)
        : world_(world), body_(body), map_voxel_size_(map_voxel_size),
          map_publish_period_(map_publish_period)
    {
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vilo/pose", 10, true);
        lidar_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vilo/lidar_pose", 10, true);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/vilo/odom", 10, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/vilo/path", 1, true);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vilo/map_cloud", 1, true);
        clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10, true);
        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic, 1, true);
        if (!right_image_topic.empty())
            right_image_pub_ = nh_.advertise<sensor_msgs::Image>(right_image_topic, 1, true);
        // Retain the last dynamic transform for viewers joining after the bag
        // ends. Latching delivers it only to new subscribers, not repeatedly
        // to listeners that already have this sensor timestamp.
        tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100, true);
        std::vector<geometry_msgs::TransformStamped> transforms;
        transforms.push_back(transform(Tbc, ros::Time(0), body_, "vilo_camera"));
        transforms.push_back(transform(Tbl, ros::Time(0), body_, "vilo_lidar"));
        static_tf_.sendTransform(transforms);
    }

    void clock(const ros::Time& time)
    {
        rosgraph_msgs::Clock msg;
        msg.clock = time;
        // The core uses double seconds. Restore exact image nanoseconds when
        // emitting its image-aligned states, rather than rounding twice.
        image_stamps_[time.toSec()] = time;
        clock_pub_.publish(msg);
        ros::spinOnce();
    }

    void publishImage(const sensor_msgs::ImageConstPtr& image, bool right = false)
    {
        // These are the original bag messages, including optical frame, image
        // encoding and exposure timestamp. Image display does not require TF.
        if (right) right_image_pub_.publish(image);
        else image_pub_.publish(image);
    }

    void publish(const std::vector<vilo::LidarPtr>& states, bool update_map,
                 bool force_map = false)
    {
        if (states.empty()) return;
        const ros::Time stamp = sensorStamp(states.back()->getTimeStamp());
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = world_;
        pose.header.stamp = stamp;
        const auto Twb = states.back()->getImuPose();
        pose.pose = toPose(Twb);
        pose_pub_.publish(pose);
        pose.pose = toPose(states.back()->getLidarPose().inverse());
        lidar_pub_.publish(pose);

        nav_msgs::Odometry odom;
        odom.header = pose.header;
        odom.child_frame_id = body_;
        odom.pose.pose = toPose(Twb);
        // Core velocity is expressed in world, Odometry twist in child frame.
        const Eigen::Vector3d velocity_body = Twb.rotation_matrix().transpose()
                                             * states.back()->getVelocity();
        odom.twist.twist.linear.x = velocity_body.x();
        odom.twist.twist.linear.y = velocity_body.y();
        odom.twist.twist.linear.z = velocity_body.z();
        odom_pub_.publish(odom);
        if (!have_pose_ || stamp > last_tf_stamp_) {
            tf2_msgs::TFMessage msg;
            msg.transforms.push_back(transform(Twb, stamp, world_, body_));
            tf_pub_.publish(msg);
            last_tf_stamp_ = stamp;
            have_pose_ = true;
        }

        nav_msgs::Path path;
        path.header = pose.header;
        for (const auto& state : states) {
            geometry_msgs::PoseStamped p;
            p.header.frame_id = world_;
            p.header.stamp = sensorStamp(state->getTimeStamp());
            p.pose = toPose(state->getImuPose());
            path.poses.push_back(p);
        }
        path_pub_.publish(path);
        const auto now = ros::WallTime::now();
        if (update_map && (force_map || last_map_publish_.isZero() ||
                (now-last_map_publish_).toSec() >= map_publish_period_)) {
            publishMap(states, pose.header);
            last_map_publish_ = ros::WallTime::now();
        }
        ros::spinOnce();
    }

    void keepAlive()
    {
        ROS_INFO("Bag processing finished; keeping final SLAM topics available. Ctrl-C to exit.");
        ros::spin();
    }

    size_t mapPoints() const { return map_points_; }
    size_t mapInputPoints() const { return map_input_points_; }

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_, lidar_pub_, odom_pub_, path_pub_, map_pub_, clock_pub_, tf_pub_;
    ros::Publisher image_pub_, right_image_pub_;
    tf2_ros::StaticTransformBroadcaster static_tf_;
    ros::Time last_tf_stamp_;
    std::string world_, body_;
    bool have_pose_ = false;
    size_t map_points_ = 0;
    size_t map_input_points_ = 0;
    double map_voxel_size_, map_publish_period_;
    ros::WallTime last_map_publish_;
    std::map<double,ros::Time> image_stamps_;

    ros::Time sensorStamp(double time) const
    {
        const auto image = image_stamps_.find(time);
        if (image != image_stamps_.end()) return image->second;
        ros::Time stamp;
        stamp.fromSec(time);
        return stamp;
    }

    static geometry_msgs::Pose toPose(const Sophus::SE3& T)
    {
        if (!T.matrix().allFinite()) throw std::runtime_error("Non-finite SLAM pose");
        geometry_msgs::Pose p;
        p.position.x=T.translation().x(); p.position.y=T.translation().y(); p.position.z=T.translation().z();
        const auto q=T.unit_quaternion().normalized();
        p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
        return p;
    }

    static geometry_msgs::TransformStamped transform(const Sophus::SE3& T,
          const ros::Time& stamp, const std::string& parent, const std::string& child)
    {
        geometry_msgs::TransformStamped tf;
        tf.header.stamp=stamp; tf.header.frame_id=parent; tf.child_frame_id=child;
        const auto p=toPose(T);
        tf.transform.translation.x=p.position.x;
        tf.transform.translation.y=p.position.y;
        tf.transform.translation.z=p.position.z;
        tf.transform.rotation=p.orientation;
        return tf;
    }

    void publishMap(const std::vector<vilo::LidarPtr>& states, const std_msgs::Header& header)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& state : states) {
            if (state->id_ != 0 && !state->is_keylidar_) continue;
            const auto Twl=state->getLidarPose().inverse();
            if (!Twl.matrix().allFinite()) throw std::runtime_error("Non-finite keyframe pose");
            for (const auto& p : state->ptr_less_surfs_->points) {
                const Eigen::Vector3d q=Twl*Eigen::Vector3d(p.x,p.y,p.z);
                if (!q.allFinite()) throw std::runtime_error("Non-finite SLAM map point");
                points->push_back(pcl::PointXYZ(q.x(),q.y(),q.z()));
            }
        }
        // Fuse across keyframes, not just within each scan. This is a display
        // map resolution and does not alter feature extraction/registration.
        map_input_points_=points->size();
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize(map_voxel_size_,map_voxel_size_,map_voxel_size_);
        filter.setInputCloud(points);
        pcl::PointCloud<pcl::PointXYZ> fused;
        filter.filter(fused);
        sensor_msgs::PointCloud2 msg;
        msg.header=header;
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1,"xyz");
        modifier.resize(fused.size());
        sensor_msgs::PointCloud2Iterator<float> x(msg,"x"), y(msg,"y"), z(msg,"z");
        for (const auto& p : fused) { *x=p.x; *y=p.y; *z=p.z; ++x; ++y; ++z; }
        msg.is_dense=true;
        map_points_=fused.size();
        map_pub_.publish(msg);
    }
};
