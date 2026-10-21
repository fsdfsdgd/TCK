#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "vilo/frame_handler_mono.h"
#include "vilo/imu.h"
#include "vilo/map.h"
#include "ros_bag_output.h"

struct ImageSample
{
    double t;
    sensor_msgs::ImageConstPtr message;
};

struct ImuSample
{
    double t;
    double ax, ay, az;
    double gx, gy, gz;
};

struct PointSample
{
    uint32_t t_ns;
    pcl::PointXYZI point;
};

struct LidarSample
{
    double t;
    double start;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<double> point_times;
};

static bool searchImu(
    const std::vector<vilo::IMUPoint>& imu,
    double last_t, double current_t, int& start, int& end)
{
    start = -1;
    end = -1;

    for (int i = 0; i < static_cast<int>(imu.size()); ++i)
    {
        if (imu[i].t <= last_t) start = i;
        if (imu[i].t >= current_t) { end = i; break; }
    }

    return current_t > last_t && start >= 0 && end > start;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_vilo_bag");
    ros::Time::init();

    std::string bag_path;
    std::string calib = "example/newer_college.yaml";

    std::string image_topic = "/camera/infra1/image_rect_raw";
    std::string right_image_topic = "/camera/infra2/image_rect_raw";
    std::string lidar_topic = "/os1_cloud_node/points";
    std::string imu_topic = "/os1_cloud_node/imu";

    double offset = 0.0;
    double duration = 8.0;
    double map_voxel_size = 0.2, map_publish_period = 1.0;
    std::string output_path = "trajectory_lidar.tum";
    bool publish_ros = false, keep_alive = false;
    std::string world_frame = "vilo_world", body_frame = "vilo_body";

    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        const auto pos = arg.find('=');
        if (pos == std::string::npos) continue;

        const std::string key = arg.substr(0, pos);
        const std::string value = arg.substr(pos + 1);

        if (key == "bag") bag_path = value;
        else if (key == "calib") calib = value;
        else if (key == "image_topic") image_topic = value;
        else if (key == "right_image_topic") right_image_topic = value;
        else if (key == "lidar_topic") lidar_topic = value;
        else if (key == "imu_topic") imu_topic = value;
        else if (key == "offset") offset = std::stod(value);
        else if (key == "duration") duration = std::stod(value);
        else if (key == "map_voxel_size") map_voxel_size = std::stod(value);
        else if (key == "map_publish_period") map_publish_period = std::stod(value);
        else if (key == "output") output_path = value;
        else if (key == "publish") publish_ros = value == "true" || value == "1";
        else if (key == "keep_alive") keep_alive = value == "true" || value == "1";
        else if (key == "world_frame") world_frame = value;
        else if (key == "body_frame") body_frame = value;
    }

    if (bag_path.empty() || offset < 0 || duration <= 0 ||
        !std::isfinite(map_voxel_size) || map_voxel_size <= 0 ||
        !std::isfinite(map_publish_period) || map_publish_period < 0)
    {
        std::cerr
            << "Usage: test_vilo_bag bag=FILE.bag "
            << "calib=FILE.yaml offset=0 duration=8 "
            << "map_voxel_size=0.2 map_publish_period=1.0\n";
        return 1;
    }

    std::vector<ImageSample> images;
    std::vector<LidarSample> lidars;
    std::vector<ImuSample> imus;
    ros::Time begin, end;

    try
    {
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);

        rosbag::View all_view(bag);
        double begin_sec = all_view.getBeginTime().toSec() + offset;
        double end_sec = begin_sec + duration;

        begin.fromSec(begin_sec);
        end.fromSec(end_sec);

        std::vector<std::string> topics = {
            image_topic, lidar_topic, imu_topic
        };

        rosbag::View view(
            bag, rosbag::TopicQuery(topics), begin, end);

        std::cout << "Reading Bag: " << bag_path << '\n';
        std::cout << "Time window: " << begin_sec
                  << " - " << end_sec << '\n';

        for (const rosbag::MessageInstance& item : view)
        {
            const std::string topic = item.getTopic();

            if (topic == image_topic)
            {
                auto msg = item.instantiate<sensor_msgs::Image>();
                if (!msg) continue;

                if (msg->encoding !=
                    sensor_msgs::image_encodings::MONO8)
                {
                    std::cerr << "Unsupported image encoding: "
                              << msg->encoding << '\n';
                    continue;
                }

                if (msg->data.size() <
                    static_cast<size_t>(msg->step) * msg->height)
                    continue;

                ImageSample sample;
                sample.t = msg->header.stamp.toSec();
                // Retain one owned image buffer for both SLAM and ROS display.
                // The visual frontend clones its input before building frames.
                sample.message = msg;

                if (sample.t > 0)
                    images.push_back(std::move(sample));
            }
            else if (topic == imu_topic)
            {
                auto msg = item.instantiate<sensor_msgs::Imu>();
                if (!msg) continue;

                ImuSample sample;
                sample.t = msg->header.stamp.toSec();
                sample.ax = msg->linear_acceleration.x;
                sample.ay = msg->linear_acceleration.y;
                sample.az = msg->linear_acceleration.z;
                sample.gx = msg->angular_velocity.x;
                sample.gy = msg->angular_velocity.y;
                sample.gz = msg->angular_velocity.z;

                if (sample.t > 0)
                    imus.push_back(sample);
            }
            else if (topic == lidar_topic)
            {
                auto msg = item.instantiate<sensor_msgs::PointCloud2>();
                if (!msg) continue;

                LidarSample sample;
                sample.start = msg->header.stamp.toSec();
                sample.cloud.reset(
                    new pcl::PointCloud<pcl::PointXYZI>);

                std::vector<PointSample> points;
                points.reserve(
                    static_cast<size_t>(msg->width) * msg->height);

                try
                {
                    sensor_msgs::PointCloud2ConstIterator<float>
                        x(*msg, "x"), y(*msg, "y"), z(*msg, "z");

                    sensor_msgs::PointCloud2ConstIterator<uint32_t>
                        t(*msg, "t");

                    sensor_msgs::PointCloud2ConstIterator<uint8_t>
                        ring(*msg, "ring");

                    const size_t n =
                        static_cast<size_t>(msg->width) * msg->height;

                    for (size_t i = 0; i < n;
                         ++i, ++x, ++y, ++z, ++t, ++ring)
                    {
                        if (!std::isfinite(*x) ||
                            !std::isfinite(*y) ||
                            !std::isfinite(*z))
                            continue;

                        if (*x == 0 && *y == 0 && *z == 0)
                            continue;

                        PointSample p;
                        p.t_ns = *t;
                        p.point.x = *x;
                        p.point.y = *y;
                        p.point.z = *z;

                        // The current TCK-VILO lidar pipeline
                        // recalculates scan-row information itself.
                        p.point.intensity = static_cast<float>(*t) * 1e-9f;

                        points.push_back(p);
                    }
                }
                catch (const std::runtime_error& e)
                {
                    std::cerr << "Invalid PointCloud2 fields: "
                              << e.what() << '\n';
                    continue;
                }

                // TCK-VILO deskewing expects point times to
                // follow the scan's temporal order.
                std::stable_sort(
                    points.begin(), points.end(),
                    [](const PointSample& a, const PointSample& b)
                    {
                        return a.t_ns < b.t_ns;
                    });

                for (const auto& p : points)
                {
                    sample.cloud->points.push_back(p.point);
                    sample.point_times.push_back(
                        static_cast<double>(p.t_ns) * 1e-9);
                }

                if (sample.cloud->points.size() >= 10)
                {
                    sample.t = sample.start + sample.point_times.back();
                    sample.cloud->width =
                        static_cast<uint32_t>(
                            sample.cloud->points.size());
                    sample.cloud->height = 1;
                    sample.cloud->is_dense = true;
                    lidars.push_back(std::move(sample));
                }
            }
        }

        bag.close();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Bag read failed: " << e.what() << '\n';
        return 2;
    }

    auto by_time = [](const auto& a, const auto& b)
    {
        return a.t < b.t;
    };

    std::sort(images.begin(), images.end(), by_time);
    std::sort(lidars.begin(), lidars.end(), by_time);
    std::sort(imus.begin(), imus.end(), by_time);

    std::cout << "Images: " << images.size() << '\n';
    std::cout << "Lidars: " << lidars.size() << '\n';
    std::cout << "IMUs:   " << imus.size() << '\n';

    if (images.size() < 10 || lidars.size() < 2 ||
        imus.size() < 10)
    {
        std::cerr << "Not enough synchronized sensor messages.\n";
        return 3;
    }

    vilo::FrameHandlerMono vo(calib, false);
    vo.start();
    std::unique_ptr<RosBagOutput> ros_output;
    if (publish_ros)
        ros_output.reset(new RosBagOutput(world_frame,body_frame,vo.T_b_c_,vo.T_b_l_,
                                         image_topic,right_image_topic,
                                         map_voxel_size,map_publish_period));

    // Stream the display-only right camera rather than retaining another full
    // image sequence in memory. It does not enter the monocular SLAM frontend.
    rosbag::Bag right_bag;
    std::unique_ptr<rosbag::View> right_view;
    rosbag::View::iterator right_it;
    sensor_msgs::ImageConstPtr next_right;
    if (ros_output && !right_image_topic.empty()) {
        try {
            right_bag.open(bag_path, rosbag::bagmode::Read);
            right_view.reset(new rosbag::View(right_bag,
                rosbag::TopicQuery(std::vector<std::string>{right_image_topic}),begin,end));
            right_it = right_view->begin();
            if (right_it == right_view->end())
                ROS_WARN_STREAM("No right camera images in selected bag window: " << right_image_topic);
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Cannot open right image stream: " << e.what());
            return 2;
        }
    }

    std::vector<vilo::IMUPoint> imu_vec;
    for (const auto& m : imus)
        imu_vec.emplace_back(Eigen::Vector3d(m.ax,m.ay,m.az),
                             Eigen::Vector3d(m.gx,m.gy,m.gz),m.t);
    std::ofstream output(output_path);
    if (!output.is_open())
    {
        std::cerr << "Cannot create " << output_path << '\n';
        return 4;
    }
    output << std::fixed << std::setprecision(9);
    double last_image_ts = 0.0;
    double last_lidar_ts = 0.0;
    size_t lidar_id = 0, processed_images = 0, processed_lidars = 0;
    bool lidar_started = false;
    std::vector<vilo::LidarPtr> trajectory;
    for (auto& image : images) {
        if (publish_ros && !ros::ok()) break;
        const double image_ts = image.t;
        if (ros_output) {
            ros_output->clock(image.message->header.stamp);
            // Publish even while stationary/before initialization, with no
            // future images relative to the estimator's simulated clock.
            ros_output->publishImage(image.message);
            while (right_view && right_it != right_view->end()) {
                if (!next_right) next_right = right_it->instantiate<sensor_msgs::Image>();
                if (!next_right) { ++right_it; continue; }
                if (next_right->header.stamp > image.message->header.stamp) break;
                ros_output->publishImage(next_right,true);
                next_right.reset();
                ++right_it;
            }
        }
        if (image_ts <= imus.front().t || image_ts >= imus.back().t) continue;
        if (!last_image_ts) { last_image_ts = image_ts; continue; }
        int si=-1,ei=-1,sl=-1,el=-1;
        if (!searchImu(imu_vec,last_image_ts,image_ts,si,ei)) continue;
        if (!lidar_started && vo.stage() == vilo::FrameHandlerBase::STAGE_DEFAULT_FRAME) {
            lidar_started = true;
            last_lidar_ts = last_image_ts;
            std::cout << "Visual bootstrap complete; enabling LiDAR\n";
        }
        while (lidar_id < lidars.size() && lidars[lidar_id].t <= last_lidar_ts)
            ++lidar_id;
        const bool use_lidar = lidar_started && lidar_id < lidars.size() &&
                              lidars[lidar_id].t <= image_ts;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<double> point_times;
        double lidar_ts = last_lidar_ts;
        if (use_lidar) {
            auto& scan = lidars[lidar_id];
            const double state_time = vo.is_lio_initialized_ ? image_ts : scan.t;
            if (!searchImu(imu_vec,last_lidar_ts,state_time,sl,el)) {
                std::cerr << "Missing bracketed LiDAR IMU interval\n";
                return 5;
            }
            cloud = scan.cloud;
            point_times = scan.point_times;
            if (vo.is_lio_initialized_)
                for (auto& t : point_times) t += scan.start - last_lidar_ts;
            lidar_ts = scan.t;
        }
        const auto& message = image.message;
        cv::Mat input_image(message->height,message->width,CV_8UC1,
                            const_cast<uint8_t*>(message->data.data()),message->step);
        vo.addLidarImageImu(cloud,input_image,imu_vec,point_times,
                            lidar_ts,image_ts,si,ei,sl,el,use_lidar?1:0);
        vo.depthFilter()->waitUntilIdle();
        last_image_ts = image_ts;
        ++processed_images;
        if (use_lidar) {
            ++processed_lidars;
            ++lidar_id;
            if (vo.lastLidar()) {
                last_lidar_ts = vo.lastLidar()->getTimeStamp();
                trajectory.push_back(vo.lastLidar());
                if (ros_output) ros_output->publish(trajectory,processed_lidars%5 == 1);
            }
        }
        if (processed_images % 30 == 0)
            std::cout << "PROGRESS images=" << processed_images << " lidars=" << processed_lidars
                      << " initialized[vio=" << vo.is_vio_initialized_ << " lio="
                      << vo.is_lio_initialized_ << " fused=" << vo.is_livo_initialized_ << "]\n";
    }
    vo.depthFilter()->waitUntilIdle();
    for (const auto& lidar : trajectory) {
        const auto pose = lidar->getImuPose();
        if (!pose.matrix().allFinite()) { std::cerr << "Non-finite trajectory pose\n"; return 6; }
        const auto q=pose.unit_quaternion();
        output << lidar->getTimeStamp() << " " << pose.translation().transpose() << " "
               << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << '\n';
    }
    output.close();
    if (ros_output) ros_output->publish(trajectory,true,true);
    std::cout << "RESULT images=" << processed_images << " lidars=" << processed_lidars
              << " trajectory=" << trajectory.size() << " initialized[vio="
              << vo.is_vio_initialized_ << " lio=" << vo.is_lio_initialized_
              << " fused=" << vo.is_livo_initialized_ << "] map_points="
              << (ros_output ? ros_output->mapPoints() : 0)
              << " map_input_points=" << (ros_output ? ros_output->mapInputPoints() : 0)
              << " output=" << output_path << std::endl;
    if (trajectory.empty())
        std::cerr << "No initialized SLAM trajectory in this window; extend duration to include motion.\n";
    if (ros_output && keep_alive) ros_output->keepAlive();
    return trajectory.empty() ? 7 : 0;
}
