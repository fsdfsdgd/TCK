#include <iostream>
#include <stdexcept>
#include "vilo/imu_timeline.h"
#include "vilo/depth_filter.h"
#include "vilo/sensor_reliability.h"
#include "vilo/initialization_alignment.h"
#include "vilo/camera.h"
#include "vilo/frame.h"
#include "../example/incremental_voxel_map.h"
#include "../example/slam_archive.h"

static void require(bool ok,const char* message) {
    if(!ok)throw std::runtime_error(message);
}

int main() {
    try {
        std::vector<Sophus::SE3> visual_poses, lidar_poses;
        const Eigen::Matrix3d world_rotation = Sophus::SO3::exp(Eigen::Vector3d(0,0,2.8)).matrix();
        const Eigen::Vector3d translation(2,-3,1);
        for (int i=0;i<19;++i) {
            const Eigen::Vector3d p(0.01*i,0.04*i,0.001*std::sin(i));
            const Eigen::Matrix3d r = Sophus::SO3::exp(Eigen::Vector3d(.01*i,.005*i,.02*i)).matrix();
            visual_poses.emplace_back(r,p);
            lidar_poses.emplace_back(world_rotation*r,7.4*world_rotation*p+translation);
        }
        const auto alignment = vilo::alignInitializationPoses(visual_poses,lidar_poses);
        require(alignment.accepted && std::abs(alignment.scale-7.4)<1e-10 &&
                (alignment.rotation-world_rotation).norm()<1e-10 && alignment.position_rmse<1e-10,
                "Moving, nearly straight initialization collapsed or changed body axes");
        std::vector<Sophus::SE3> stationary(19,Sophus::SE3());
        require(!vilo::alignInitializationPoses(stationary,stationary).accepted,
                "Unobservable stationary scale accepted");
        auto inconsistent=lidar_poses;
        inconsistent.back()=Sophus::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(50,20,10));
        require(!vilo::alignInitializationPoses(visual_poses,inconsistent).accepted,
                "Inconsistent initialization accepted");
        {
            vilo::PinholeCamera camera(640,480,400,400,320,240);
            cv::Mat image(480,640,CV_8UC1); cv::randu(image,0,255);
            Sophus::SE3 Tbc;
            vilo::FramePtr frame(new vilo::Frame(&camera,image,1,Tbc));
            const Sophus::SE3 original = visual_poses.back();
            auto camera_pose=original.inverse(); frame->setFramePose(camera_pose);
            frame->setVelocity(Eigen::Vector3d(1,2,3));
            vilo::Map visual_map; visual_map.addKeyframe(frame);
            Eigen::Matrix3d r=alignment.rotation;
            Eigen::Vector3d t=alignment.translation;
            double s=alignment.scale;
            visual_map.scaleRotate(Tbc,r,r,t,s);
            require((frame->getImuRotation()-r*original.rotation_matrix()).norm()<1e-10 &&
                    (frame->getImuPosition()-s*r*original.translation()-t).norm()<1e-10 &&
                    (frame->getVelocity()-s*r*Eigen::Vector3d(1,2,3)).norm()<1e-10,
                    "World alignment was applied on the wrong side or velocity was not transformed");
        }
        const auto weights=vilo::fusedSensorWeights(1000,4000,1,1,false);
        require(std::abs(weights.first-0.25*std::atan(1.0))<1e-12,
                "Integer division disabled the LiDAR constraints");
        const auto lost=vilo::fusedSensorWeights(1000,0,1,0,true);
        require(lost.first==10&&lost.second==0,"Empty visual observations divide by zero");
        require(vilo::relativeSensorReliability(1000,.1,{}, {})==1,
                "First fused window has no reliability history");
        require(vilo::relativeSensorReliability(1000,0,{1000},{.1})==
                std::numeric_limits<double>::infinity(),"Zero residual limit is invalid");
        std::vector<vilo::IMUPoint> imu;
        const Eigen::Vector3d gyro(0,0,0.8),acc(0,0,vilo::GRAVITY_VALUE);
        for(int i=0;i<18;++i)imu.emplace_back(acc,gyro,9.95+i*0.01);
        double integrated=0;
        vilo::forImuInterval(imu,10.0,10.077,[&](const Eigen::Vector3d&,const Eigen::Vector3d&,double dt){
            require(dt>0,"Negative/zero preintegration step");integrated+=dt;
        });
        require(std::abs(integrated-0.077)<1e-12,"Wrong clipped integration interval");
        const Eigen::Vector3d velocity(3,-0.4,0.1);
        const Sophus::SE3 anchor(Sophus::SO3::exp(Eigen::Vector3d(0,0,0.3)).matrix(),
                                 Eigen::Vector3d(1,2,3));
        const Sophus::SE3 Tbl(Sophus::SO3::exp(Eigen::Vector3d(0.1,0.2,0)).matrix(),
                              Eigen::Vector3d(0.05,-0.02,0.03));
        vilo::ImuTimeline timeline(imu,10.0,9.97,10.1,anchor,velocity,
                                   Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
        const Eigen::Vector3d world(12,5,1);
        const Sophus::SE3 reference=(timeline.at(10.1)*Tbl).inverse();
        double error=0;
        for(int i=0;i<=130;++i) {
            const double t=i==130?10.1:9.97+i*0.001;
            const double dt=t-10.0;
            const Sophus::SE3 exact(anchor.rotation_matrix()*Sophus::SO3::exp(gyro*dt).matrix(),
                                     anchor.translation()+velocity*dt);
            const Eigen::Vector3d observed=(exact*Tbl).inverse()*world;
            const Eigen::Vector3d corrected=reference*(timeline.at(t)*Tbl*observed);
            error=std::max(error,(corrected-reference*world).norm());
        }
        require(error<1e-9,"Deskew failed before anchor/with sensor extrinsics");
        bool rejected=false;
        try { vilo::ImuTimeline invalid(imu,10,9.8,10.1,anchor,velocity,
                Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()); }
        catch(const std::runtime_error&){rejected=true;}
        require(rejected,"Missing scan coverage was silently accepted");

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI point;point.x=0.01;point.y=-0.02;point.z=0.03;point.intensity=1;
        cloud->push_back(point);
        IncrementalVoxelMap cache(0.2);
        const Sophus::SE3 identity;
        cache.update(1,identity,cloud);cache.update(1,identity,cloud);
        require(cache.size()==1&&cache.inputPoints()==1,"Repeated keyframe increased map");
        cache.update(2,identity,cloud);
        const Sophus::SE3 moved(Eigen::Matrix3d::Identity(),Eigen::Vector3d(1,0,0));
        cache.update(1,moved,cloud);
        require(cache.size()==2,"Other keyframe contribution incorrectly removed");
        cache.update(2,moved,cloud);
        require(cache.size()==1&&cache.inputPoints()==2,"Old optimized geometry left behind");
        cache.visit([&](const Eigen::Vector3d& p){require((p-moved*Eigen::Vector3d(point.x,point.y,point.z)).norm()<1e-7,"Wrong voxel centroid");});

        SlamArchive archive;
        vilo::LidarPtr scan(new vilo::Lidar(cloud));
        scan->setTimeStamp(10.1);
        auto extrinsic=identity;scan->setTbl(extrinsic);
        boost::weak_ptr<vilo::Lidar> weak=scan;
        archive.append(scan);scan.reset();archive.refresh();
        require(weak.expired()&&archive.liveStates()==0,"Output archive retains complete scans");

        // These rays belong to hardware channel 32 even though deskew has
        // rotated them above the sensor's nominal vertical field of view.
        pcl::PointCloud<pcl::PointXYZI>::Ptr rays(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0;i<360;++i) {
            pcl::PointXYZI ray;
            ray.x=10*std::cos(i*0.0174532925199433);
            ray.y=10*std::sin(i*0.0174532925199433);
            ray.z=5;ray.intensity=32+0.05f;
            rays->push_back(ray);
        }
        vilo::Lidar registered(rays,true);
        registered.registerPoints();
        require(rays->size()>350,"Deskewed hardware rings were dropped/reassigned by elevation");
        for(const auto& ray:*rays)require(int(ray.intensity)==32,"Hardware ring was changed");

        vilo::DepthFilter mapper(nullptr,vilo::DepthFilter::callback_t{});
        vilo::LidarPtr key(new vilo::Lidar(cloud)),child(new vilo::Lidar(cloud));
        key->is_keylidar_=true;
        mapper.new_lidar_=key;mapper.last_keylidar_=key;
        mapper.addLidarSubmap(cloud);
        Sophus::SE3 child_pose(Eigen::Matrix3d::Identity(),Eigen::Vector3d(-2,0,0));
        child->setLidarPose(child_pose);
        mapper.new_lidar_=child;
        mapper.addLidarSubmap(cloud);
        Sophus::SE3 corrected_key(Eigen::Matrix3d::Identity(),Eigen::Vector3d(-10,0,0));
        key->setLidarPose(corrected_key);
        mapper.synchronizeLidarSubmaps();
        require(std::abs(child->getLidarPose().inverse().translation().x()-12)<1e-9,
                "Historical scan did not follow its optimized keyframe");

        // A biased plane moves this window. EVERY optimized state must be
        // committed, or the returned state pair violates its own IMU factor.
        vilo::LidarPtr fixed(new vilo::Lidar(cloud)),moving(new vilo::Lidar(cloud));
        float frequency=100,na=.02,ng=.002,naw=.002,ngw=.0002;
        for(auto lidar:{fixed,moving}) {
            auto transform=identity;lidar->setTbl(transform);
            lidar->setVelocity(Eigen::Vector3d(10,0,0));
            lidar->key_imu_owner_=std::make_shared<vilo::IMU>(frequency,na,ng,naw,ngw);
            lidar->imu_from_last_keylidar_=lidar->key_imu_owner_.get();
            for(int i=0;i<10;++i)lidar->key_imu_owner_->addImuPoint(acc,Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),.01);
        }
        auto first_pose=identity;fixed->setLidarPose(first_pose);
        Sophus::SE3 second_pose(Eigen::Matrix3d::Identity(),Eigen::Vector3d(-1,0,0));
        moving->setLidarPose(second_pose);
        moving->last_key_lidar_=fixed.get();
        std::list<vilo::LidarPtr> window{fixed,moving};
        std::vector<std::vector<Eigen::Vector3d>> observations(2),normals(2);
        std::vector<std::vector<double>> distances(2);
        for(int i=0;i<10;++i) {
            observations[1].push_back(Eigen::Vector3d::Zero());
            normals[1].push_back(Eigen::Vector3d::UnitX());distances[1].push_back(-2);
        }
        vilo::Map map;
        vilo::ba::lidarImuLocalBundleAdjustment(&map,window,observations,normals,distances,true);
        const auto pre=moving->imu_from_last_keylidar_;
        const double dt=pre->delta_t_;
        const Eigen::Vector3d inertial_residual=fixed->getImuRotation().transpose()*
            (moving->getImuPosition()-fixed->getImuPosition()-fixed->getVelocity()*dt-
             0.5*Eigen::Vector3d(0,0,-vilo::GRAVITY_VALUE)*dt*dt)-
            pre->getDeltaPosition(fixed->ba_,fixed->bg_);
        require(inertial_residual.norm()<1e-3,"BA solution discarded an optimized oldest state");
        std::cout<<"PASS clipped IMU, pre-anchor deskew/extrinsics error="<<error
                 <<", optimized-keyframe replacement, non-owning scan archive, anchored submap/child poses, hardware rings, complete BA writeback, fused reliability weights\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
