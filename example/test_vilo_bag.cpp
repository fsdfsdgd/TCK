#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sys/resource.h>

#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "vilo/frame_handler_mono.h"
#include "vilo/imu.h"
#include "vilo/map.h"
#include "ros_bag_output.h"

struct LidarSample {
    double t=0, start=0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<double> point_times;
};

static bool searchImu(const std::vector<vilo::IMUPoint>& imu,
                      double first,double last,int& start,int& end)
{
    auto after=std::upper_bound(imu.begin(),imu.end(),first,
        [](double t,const vilo::IMUPoint& p){return t<p.t;});
    auto finish=std::lower_bound(imu.begin(),imu.end(),last,
        [](const vilo::IMUPoint& p,double t){return p.t<t;});
    if (last<=first || after==imu.begin() || finish==imu.end()) return false;
    start=int(after-imu.begin())-1;
    end=int(finish-imu.begin());
    return end>start;
}

static bool decodeScan(const sensor_msgs::PointCloud2& msg,LidarSample& scan)
{
    struct Point { uint32_t time; pcl::PointXYZI value; };
    std::vector<Point> points;
    const size_t count=size_t(msg.width)*msg.height;
    points.reserve(count);
    sensor_msgs::PointCloud2ConstIterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> t(msg,"t");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ring(msg,"ring");
    for (size_t i=0;i<count;++i,++x,++y,++z,++t,++ring) {
        if (!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z) ||
            (*x==0&&*y==0&&*z==0)) continue;
        Point point;
        point.time=*t;
        point.value.x=*x; point.value.y=*y; point.value.z=*z;
        if(*ring>=64)throw std::runtime_error("Expected an Ouster-64 hardware ring in [0,63]");
        point.value.intensity=float(*ring)+float(*t)*1e-9f;
        points.push_back(point);
    }
    if (points.size()<11) return false;
    std::stable_sort(points.begin(),points.end(),
        [](const Point& a,const Point& b){return a.time<b.time;});
    scan=LidarSample();
    scan.start=msg.header.stamp.toSec();
    scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    scan.cloud->reserve(points.size());
    scan.point_times.reserve(points.size());
    for (const auto& point:points) {
        scan.cloud->push_back(point.value);
        scan.point_times.push_back(double(point.time)*1e-9);
    }
    scan.t=scan.start+scan.point_times.back();
    scan.cloud->is_dense=true;
    return scan.start>0;
}

static double peakMemoryMB()
{
    struct rusage usage{};
    getrusage(RUSAGE_SELF,&usage);
    return usage.ru_maxrss/1024.0;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_vilo_bag");
    ros::Time::init();
    std::string bag_path,calib="example/newer_college.yaml";
    std::string image_topic="/camera/infra1/image_rect_raw";
    std::string right_image_topic="/camera/infra2/image_rect_raw";
    std::string lidar_topic="/os1_cloud_node/points",imu_topic="/os1_cloud_node/imu";
    std::string output_path="trajectory_lidar.tum",world="vilo_world",body="vilo_body";
    double offset=0,duration=8,map_voxel_size=0.2,map_publish_period=1;
    bool publish=false,keep_alive=false;
    for (int i=1;i<argc;++i) {
        std::string arg=argv[i];
        auto pos=arg.find('=');
        if (pos==std::string::npos) continue;
        auto key=arg.substr(0,pos),value=arg.substr(pos+1);
        if(key=="bag")bag_path=value;
        else if(key=="calib")calib=value;
        else if(key=="image_topic")image_topic=value;
        else if(key=="right_image_topic")right_image_topic=value;
        else if(key=="lidar_topic")lidar_topic=value;
        else if(key=="imu_topic")imu_topic=value;
        else if(key=="output")output_path=value;
        else if(key=="world_frame")world=value;
        else if(key=="body_frame")body=value;
        else if(key=="offset")offset=std::stod(value);
        else if(key=="duration")duration=std::stod(value);
        else if(key=="map_voxel_size")map_voxel_size=std::stod(value);
        else if(key=="map_publish_period")map_publish_period=std::stod(value);
        else if(key=="publish")publish=value=="true"||value=="1";
        else if(key=="keep_alive")keep_alive=value=="true"||value=="1";
    }
    if(bag_path.empty()||!std::isfinite(offset)||offset<0||
       !std::isfinite(duration)||duration<=0||!std::isfinite(map_voxel_size)||
       map_voxel_size<=0||!std::isfinite(map_publish_period)||map_publish_period<0) {
        std::cerr<<"Usage: test_vilo_bag bag=FILE calib=FILE offset=0 duration=8\n";
        return 1;
    }
    try {
        rosbag::Bag bag;
        bag.open(bag_path,rosbag::bagmode::Read);
        rosbag::View all(bag);
        ros::Time begin,end,imu_begin,imu_end;
        begin.fromSec(all.getBeginTime().toSec()+offset);
        end.fromSec(begin.toSec()+duration);
        imu_begin.fromSec(std::max(0.0,begin.toSec()-0.2));
        imu_end.fromSec(end.toSec()+0.2);
        std::cout<<"Streaming Bag: "<<bag_path<<"\nTime window: "
                 <<std::fixed<<std::setprecision(6)<<begin.toSec()<<" - "<<end.toSec()<<'\n';
        // IMU is small; retain it for bracketed random-access integration.
        // Images/clouds are decoded only at the current processing cursor.
        std::vector<vilo::IMUPoint> imu;
        rosbag::View imu_view(bag,rosbag::TopicQuery(std::vector<std::string>{imu_topic}),
                             imu_begin,imu_end);
        for(const auto& item:imu_view) {
            auto msg=item.instantiate<sensor_msgs::Imu>();
            if(!msg||msg->header.stamp.isZero())continue;
            imu.emplace_back(Eigen::Vector3d(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z),
                             Eigen::Vector3d(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z),
                             msg->header.stamp.toSec());
        }
        std::stable_sort(imu.begin(),imu.end(),
            [](const vilo::IMUPoint& a,const vilo::IMUPoint& b){return a.t<b.t;});
        if(imu.size()<10)throw std::runtime_error("Not enough IMU samples");
        for(size_t i=1;i<imu.size();++i)
            if(imu[i].t<=imu[i-1].t)throw std::runtime_error("Non-increasing IMU sensor timestamps");

        rosbag::View images(bag,rosbag::TopicQuery(std::vector<std::string>{image_topic}),begin,end);
        rosbag::View lidars(bag,rosbag::TopicQuery(std::vector<std::string>{lidar_topic}),begin,end);
        if(images.size()<10||lidars.size()<2)throw std::runtime_error("Missing image/LiDAR topic or short bag window");
        std::cout<<"Images: "<<images.size()<<"\nLidars: "<<lidars.size()<<"\nIMUs: "<<imu.size()<<'\n';

        vilo::FrameHandlerMono vo(calib,false);
        vo.start();
        std::unique_ptr<RosBagOutput> output_ros;
        if(publish)output_ros.reset(new RosBagOutput(world,body,vo.T_b_c_,vo.T_b_l_,
            image_topic,right_image_topic,map_voxel_size,map_publish_period));
        std::unique_ptr<rosbag::View> right_view;
        rosbag::View::iterator right_it;
        sensor_msgs::ImageConstPtr next_right;
        if(output_ros&&!right_image_topic.empty()) {
            right_view.reset(new rosbag::View(bag,
                rosbag::TopicQuery(std::vector<std::string>{right_image_topic}),begin,end));
            right_it=right_view->begin();
            if(right_it==right_view->end())ROS_WARN_STREAM("No right image topic: "<<right_image_topic);
        }
        std::ofstream trajectory_file(output_path);
        if(!trajectory_file)throw std::runtime_error("Cannot create trajectory output: "+output_path);
        SlamArchive archive;
        auto lidar_it=lidars.begin();
        LidarSample pending;
        double last_read_lidar=0;
        auto next_scan=[&]() {
            while(lidar_it!=lidars.end()) {
                auto message=lidar_it->instantiate<sensor_msgs::PointCloud2>();
                ++lidar_it;
                if(!message||!decodeScan(*message,pending))continue;
                if(pending.t<=last_read_lidar)throw std::runtime_error("Non-increasing LiDAR sensor timestamps");
                last_read_lidar=pending.t;
                return true;
            }
            pending=LidarSample();
            return false;
        };
        bool have_scan=next_scan(),lidar_started=false;
        double last_image=0,last_lidar=0,last_read_image=0;
        size_t processed_images=0,processed_lidars=0;
        for(const auto& item:images) {
            if(publish&&!ros::ok())break;
            auto image=item.instantiate<sensor_msgs::Image>();
            if(!image)continue;
            if(image->encoding!=sensor_msgs::image_encodings::MONO8 ||
               image->step<image->width || image->data.size()<size_t(image->step)*image->height)
                throw std::runtime_error("Expected valid MONO8 SLAM image");
            const double time=image->header.stamp.toSec();
            if(time<=last_read_image)throw std::runtime_error("Non-increasing image sensor timestamps");
            last_read_image=time;
            if(output_ros) {
                output_ros->clock(image->header.stamp);
                output_ros->publishImage(image);
                while(right_view&&right_it!=right_view->end()) {
                    if(!next_right)next_right=right_it->instantiate<sensor_msgs::Image>();
                    if(!next_right){++right_it;continue;}
                    if(next_right->header.stamp>image->header.stamp)break;
                    output_ros->publishImage(next_right,true);
                    next_right.reset();++right_it;
                }
            }
            if(time<=imu.front().t||time>=imu.back().t)continue;
            if(!last_image){last_image=time;continue;}
            int si=-1,ei=-1,sl=-1,el=-1;
            if(!searchImu(imu,last_image,time,si,ei))
                throw std::runtime_error("Missing image IMU coverage");
            if(!lidar_started&&vo.stage()==vilo::FrameHandlerBase::STAGE_DEFAULT_FRAME) {
                lidar_started=true;last_lidar=last_image;
                std::cout<<"Visual bootstrap complete; enabling LiDAR\n";
            }
            while(have_scan&&pending.t<=last_lidar)have_scan=next_scan();
            bool use_lidar=lidar_started&&have_scan&&pending.t<=time;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            std::vector<double> point_times;
            double scan_time=last_lidar;
            if(use_lidar) {
                const double target=vo.is_lio_initialized_?time:pending.t;
                const double first=vo.is_lio_initialized_?
                    std::min(last_lidar,pending.start+pending.point_times.front()):last_lidar;
                if(!searchImu(imu,first,target,sl,el))
                    throw std::runtime_error("Missing full-scan LiDAR IMU coverage");
                cloud=pending.cloud;point_times=pending.point_times;scan_time=pending.t;
                if(vo.is_lio_initialized_)
                    for(auto& t:point_times)t+=pending.start-last_lidar;
            }
            cv::Mat input(image->height,image->width,CV_8UC1,
                          const_cast<uint8_t*>(image->data.data()),image->step);
            vo.addLidarImageImu(cloud,input,imu,point_times,scan_time,time,
                                si,ei,sl,el,use_lidar?1:0,true);
            vo.depthFilter()->waitUntilIdle();
            // Consumers of m_last_frame need the immediate predecessor only.
            // Keep that predecessor, but stop retaining its entire ancestor chain.
            if(vo.lastFrame()&&vo.lastFrame()->m_last_frame)
                vo.lastFrame()->m_last_frame->m_last_frame.reset();
            last_image=time;++processed_images;
            if(use_lidar) {
                ++processed_lidars;
                last_lidar=vo.lastLidar()->getTimeStamp();
                archive.append(vo.lastLidar());
                archive.refresh();
                if(output_ros)output_ros->publish(archive.states,processed_lidars%5==1);
                have_scan=next_scan();
            }
            if(processed_images%30==0)
                std::cout<<"PROGRESS images="<<processed_images<<" lidars="<<processed_lidars
                    <<" initialized[vio="<<vo.is_vio_initialized_<<" lio="<<vo.is_lio_initialized_
                    <<" fused="<<vo.is_livo_initialized_<<"] live_lidars="<<archive.liveStates()
                    <<" peak_rss_mb="<<peakMemoryMB()<<std::endl;
        }
        vo.depthFilter()->waitUntilIdle();
        archive.refresh();
        trajectory_file<<std::fixed<<std::setprecision(9);
        for(const auto& state:archive.states) {
            const auto q=state.Twb.unit_quaternion();
            trajectory_file<<state.time<<" "<<state.Twb.translation().transpose()<<" "
                <<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<'\n';
        }
        trajectory_file.close();
        if(output_ros)output_ros->publish(archive.states,true,true);
        std::cout<<"RESULT images="<<processed_images<<" lidars="<<processed_lidars
            <<" trajectory="<<archive.states.size()<<" initialized[vio="<<vo.is_vio_initialized_
            <<" lio="<<vo.is_lio_initialized_<<" fused="<<vo.is_livo_initialized_
            <<"] map_points="<<(output_ros?output_ros->mapPoints():0)
            <<" map_input_points="<<(output_ros?output_ros->mapInputPoints():0)
            <<" peak_rss_mb="<<peakMemoryMB()<<" output="<<output_path<<std::endl;
        bag.close();
        const bool empty=archive.states.empty();
        const bool initialized=vo.is_vio_initialized_&&vo.is_lio_initialized_&&vo.is_livo_initialized_;
        if(empty||!initialized)
            std::cerr<<"SLAM initialization incomplete; partial poses are diagnostic output, not a successful fused trajectory.\n";
        if(output_ros&&keep_alive&&!empty&&initialized)output_ros->keepAlive();
        return empty||!initialized?7:0;
    } catch(const std::exception& error) {
        std::cerr<<"Bag processing failed: "<<error.what()<<std::endl;
        return 2;
    }
}
