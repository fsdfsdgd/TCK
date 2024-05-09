#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "vilo/config.h"
#include "vilo/frame_handler_mono.h"
#include "vilo/frame_handler_base.h"
#include "vilo/map.h"
#include "vilo/frame.h"
#include "vilo/feature.h"
#include "vilo/point.h"
#include "vilo/viewer.h"
#include "vilo/depth_filter.h"
#include "vilo/photomatric_calibration.h"
#include "vilo/camera.h"
#include "vilo/file_reader.h"
#include "vilo/imu.h"



using namespace cv;
using namespace std;

const int G_MAX_RESOLUTION = 848*800;

string g_image_folder = "";
string g_lidar_folder = "";
string g_imu_folder = "";
string g_stamp_folder = "";
string g_calib_path = "";
bool g_use_pc = false;
bool g_show_pc = false;
int g_start = 500;
int g_end = 14900;

Sophus::SE3 T_start;
std::vector<double> ts;
std::vector<Sophus::SE3> poses;


void parseArgument(char* arg)
{
    int option;
    char buf[1000];

    if(1==sscanf(arg,"images=%s",buf))
    {
        g_image_folder = buf;
        printf("loading images from %s!\n", g_image_folder.c_str());
        return;
    }

    if(1==sscanf(arg,"lidars=%s",buf))
    {
        g_lidar_folder = buf;
        printf("loading lidar point clouds from %s!\n", g_lidar_folder.c_str());
        return;
    }

    if(1==sscanf(arg,"imus=%s",buf))
    {
        g_imu_folder = buf;
        printf("loading imus from %s!\n", g_imu_folder.c_str());
        return;
    }

    if(1==sscanf(arg,"timefile=%s",buf))
    {
        g_stamp_folder = buf;
        printf("loading time stamp from %s!\n", g_stamp_folder.c_str());
        return;
    }

    if(1==sscanf(arg,"calib=%s",buf))
    {
        g_calib_path = buf;
        printf("loading calibration from %s!\n", g_calib_path.c_str());
        return;
    }

    if(1==sscanf(arg,"mode=%d",&option))
    {
        if(option==0)
        {
            g_use_pc = false;
            printf("Photometric calibration turn off!\n");
            return;
        }
        if(option==1)
        {
            g_use_pc = true;
            printf("Photometric calibration turn on!\n");
            return;
        }
    }

    if(1==sscanf(arg,"show=%d",&option))
    {
        if(option==0)
        {
            g_show_pc = false;
            return;
        }
        if(option==1)
        {
            g_show_pc = true;
            return;
        }
    }

    if(1==sscanf(arg,"start=%d",&option))
    {
        g_start = option;
        printf("START AT %d!\n",g_start);
        return;
    }

    if(1==sscanf(arg,"end=%d",&option))
    {
        g_end = option;
        printf("END AT %d!\n",g_end);
        return;
    }
}

void preprocessVelodyne(pcl::PointCloud<pcl::PointXYZI>::Ptr& pcs, std::vector<double>& pts_ts_vec, int N_SCANS)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts (new pcl::PointCloud<pcl::PointXYZI>);
    int N = pcs->points.size();
    float startOri = -atan2(pcs->points[0].y, pcs->points[0].x);
    float endOri = -atan2(pcs->points[N - 1].y,
                          pcs->points[N - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    int count = N;
    pcl::PointXYZI point;
    
    for (int i = 0; i < N; i++)
    {
        point.x = pcs->points[i].x;
        point.y = pcs->points[i].y;
        point.z = pcs->points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            if(scanID < 0 || scanID > 50)
            {
                count--;
                continue;
            }
            point.intensity = scanID;
        }
        else
        {
            printf("wrong scan number\n");
        }

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        pts_ts_vec.push_back(relTime);
        pts->points.push_back(point);
    }
    
    N = count;

    pcs->clear();
    pcs = pts;
}

namespace vilo {

class BenchmarkNode
{
public:
    vilo::AbstractCamera* cam_;
    FrameHandlerMono* vo_;
    vilo::Viewer* viewer_;
    boost::thread * viewer_thread_;

    BenchmarkNode();
    ~BenchmarkNode();
    void runFromFolder();
    void readImgWidthHeight(std::string& file, int& width, int& height);
    void saveResult(bool stamp_valid);
    bool searchHeadTailImuId(std::vector<vilo::IMUPoint>& imu_vec, 
                                double& last_ts,double& curr_ts,
                                int& s_id, int& e_id);
};


BenchmarkNode::BenchmarkNode()
{
    
}

BenchmarkNode::~BenchmarkNode()
{
    delete vo_;
    delete viewer_;
    delete viewer_thread_;
}


void BenchmarkNode::runFromFolder()
{
    vilo::FileReader file_reader; 
    
    std::vector<Eigen::Vector3d> imu_acc_vec, imu_gyro_vec;
    std::vector<double> imu_time_vec;
    long nImu = file_reader.readImuTXT(g_imu_folder,imu_acc_vec, imu_gyro_vec,imu_time_vec);

    std::vector<string> lidar_name_vec; 
    std::vector<double> lidar_time_vec;

    long nLidar = file_reader.readFileNamesTimes(g_lidar_folder,lidar_name_vec,lidar_time_vec, 0);

    std::vector<string> image_name_vec; 
    std::vector<double> image_time_vec;

    long nImg = file_reader.readFileNamesTimes(g_image_folder,image_name_vec,image_time_vec, 1);
    int width=848, height=480;
    
    vo_ = new FrameHandlerMono(g_calib_path, g_use_pc);
    vo_->start();

    viewer_ = new vilo::Viewer(vo_);
    viewer_thread_ = new boost::thread(&vilo::Viewer::run, viewer_);
    viewer_thread_->detach();

    struct timeval st,et;
    gettimeofday(&st,NULL);
    
    g_end = std::min(nImg, long(g_end) );
    long imu_id = 0;
    bool is_first_imu = false;
    long start_lidar_id = -1;
    long lidar_id = 0;
    
    for(long i=0;i<nLidar;i++)
        lidar_time_vec[i] = lidar_time_vec[i] + 0.1;

    bool is_new_lidar = false;
    std::vector<vilo::IMUPoint> imu_vec;
    double last_image_ts, last_lidar_ts;

    ofstream ot("lidarPose.txt");

    for(int id=g_start; id<g_end; ++id)
    {   
        int si_id=-1, ei_id=-1, sl_id=-1, el_id=-1;
        double image_ts = image_time_vec[id];

        if(start_lidar_id == -1)
        {
            for(long i=0; i<nLidar-1; i++)
            {
                if(image_time_vec[id]<=lidar_time_vec[i+1] && 
                image_time_vec[id]>=lidar_time_vec[i])
                {
                    start_lidar_id = i;
                    break;
                }
            }
        }
        
        if(-1 == start_lidar_id)
            continue;

        if(!is_first_imu)
        {
            if(imu_time_vec[imu_id]>=lidar_time_vec[start_lidar_id])
            {
                start_lidar_id = -1;
                continue;
            }
                
            while(imu_time_vec[imu_id] <= lidar_time_vec[start_lidar_id])
                imu_id++; 
                
            imu_id--;
            is_first_imu = true;

            last_image_ts = image_ts;
            last_lidar_ts = lidar_time_vec[start_lidar_id];
            lidar_id = start_lidar_id+1;
            continue;
        }

        if(!is_first_imu)
        {
            start_lidar_id = -1;
            continue;
        } 

        double lidar_ts = lidar_time_vec[lidar_id];

        while(imu_time_vec[imu_id] <= image_ts)
        {
            imu_vec.push_back(vilo::IMUPoint(imu_acc_vec[imu_id], imu_gyro_vec[imu_id],imu_time_vec[imu_id]));
            imu_id++; 
        }
        
        if(image_ts >= lidar_ts)
            is_new_lidar = true;
        
        if(is_new_lidar)
        {
            cv::Mat image = file_reader.readImage(id, cv::Size(width, height) );
            if(image.empty())
            {
                cerr << "Failed to load image! "<<endl;
                return;
            }

            std::vector<double> pts_ts_vec; 
            file_reader.readPointsFromTXT(lidar_id, pts_ts_vec);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcs = file_reader.getLidarPtr();

            searchHeadTailImuId(imu_vec, last_image_ts, image_ts, si_id, ei_id);
            searchHeadTailImuId(imu_vec, last_lidar_ts, lidar_ts, sl_id, el_id);
            
            vo_->addLidarImageImu(pcs, image, imu_vec, pts_ts_vec, 
                                  lidar_ts, image_ts,
                                  si_id, ei_id,
                                  sl_id, el_id, 1);
 
            lidar_id++;
            imu_id = imu_id-(imu_vec.size()-el_id);
            imu_vec.clear();
            last_image_ts = image_ts;
            last_lidar_ts = lidar_ts;
            is_new_lidar = false;

            gettimeofday(&et,NULL);
            float time_use = (et.tv_sec-st.tv_sec)*1000000+(et.tv_usec-st.tv_usec);  
        }
        else
        {
            cv::Mat image = file_reader.readImage(id, cv::Size(width, height) );
            if(image.empty())
            {
                cerr << "Failed to load image! "<<endl;
                return;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr pcs(new pcl::PointCloud<pcl::PointXYZI>);
            std::vector<double> pts_ts_vec;

            searchHeadTailImuId(imu_vec, last_image_ts, image_ts, si_id, ei_id);

            vo_->addLidarImageImu(pcs, image, imu_vec, pts_ts_vec, 
                                  lidar_ts, image_ts,
                                  si_id, ei_id,
                                  sl_id, el_id, 0);

            last_image_ts = image_ts;
            gettimeofday(&et,NULL);
            float time_use = (et.tv_sec-st.tv_sec)*1000000+(et.tv_usec-st.tv_usec); 
        }

    }

    for(auto it=vo_->map_->lid_poses_map_.begin();it!=vo_->map_->lid_poses_map_.end();++it)
    {
        ot<<it->first<<" "<<it->second.translation().transpose()<<" ";
        ot<<it->second.unit_quaternion().x()<<" ";
        ot<<it->second.unit_quaternion().y()<<" ";
        ot<<it->second.unit_quaternion().z()<<" ";
        ot<<it->second.unit_quaternion().w()<<endl;
    }
    

}

void BenchmarkNode::readImgWidthHeight(std::string& file, int& width, int& height)
{
    cv::FileStorage fSettings(g_calib_path.c_str(), cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << g_calib_path << endl;
       exit(-1);
    }
    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty() && node.isInt())
    {
        width = node.operator int();
    }
    else
    {
        std::cerr << "*Camera.weight parameter doesn't exist or is not a real number*" << std::endl;
    }

    node = fSettings["Camera.height"];
    if(!node.empty() && node.isInt())
    {
        height = node.operator int();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
    }
}

void BenchmarkNode::saveResult(bool stamp_valid)
{
    std::ofstream okt("result/KeyFrameTrajectory.txt");
    std::ofstream okt1("result/KeyFramePoses.txt");

    if(vo_->map_->keyframes_.empty())
        return;

    auto it = vo_->map_->keyframes_.begin();
    SE3 Twc0 = (*it)->T_f_w_.inverse();
    SE3 Tcb = (*it)->T_c_b_;
    SE3 Twb0 = Twc0 * Tcb;
    for(auto it = vo_->map_->keyframes_.begin(); it != vo_->map_->keyframes_.end(); ++it)
    {
        SE3 Twc = (*it)->T_f_w_.inverse();
        SE3 Tcb = (*it)->T_c_b_;
        SE3 Twb = Twc * Tcb;
        
        cout<<setprecision(19);
        okt<<setprecision(19);
        okt1<<setprecision(19);
        SE3 T = Twb0.inverse() * Twb;
        if(!stamp_valid)
        {
            okt << (*it)->id_ << " ";
            okt1<< (*it)->id_ << " ";
        }        
        else
        {
            okt << (*it)->timestamp_  << " ";
            okt1<< (*it)->id_ << " ";
        }
            

        okt << T.translation()[0] << " " 
            << T.translation()[1] << " " 
            << T.translation()[2] << " "
            << T.unit_quaternion().x() << " " 
            << T.unit_quaternion().y() << " "
            << T.unit_quaternion().z() << " "
            << T.unit_quaternion().w() << endl;

        okt1<< T.translation()[0] << " " 
            << T.translation()[1] << " " 
            << T.translation()[2] << " "
            << T.unit_quaternion().x() << " " 
            << T.unit_quaternion().y() << " "
            << T.unit_quaternion().z() << " "
            << T.unit_quaternion().w() << endl;
    }
    okt.close();
    okt1.close();

    if(g_use_pc)
    {
        std::ofstream oet("ExposureTime.txt");
        for(size_t i=0; i<vo_->m_stamp_et.size(); ++i)
            oet << vo_->m_stamp_et[i].second << endl;
        oet.close();

        vo_->photomatric_calib_->saveResult("",0);
    }
}

bool BenchmarkNode::searchHeadTailImuId(std::vector<vilo::IMUPoint>& imu_vec, 
                                double& last_ts,double& curr_ts,
                                int& s_id, int& e_id)
{
    int N = imu_vec.size();
    for(int i=0;i<N;i++)
    {
        if(imu_vec[i].t<=last_ts)
            s_id = i;
        if(imu_vec[i].t<=curr_ts)
            e_id = i;
    }
    if(-1!=s_id && -1!=e_id)
        return true;
    else
        return false;
}

} 


int main(int argc, char** argv)
{    
    if(argc < 5)
    {
        cerr << endl << "Minimal Usage: ./test_vilo  lidars=PATH_TO_LIDAR_FOLDER images=PATH_TO_IMAGE_FOLDER imus=PATH_TO_IMU_FILE calib=PATH_TO_CALIBRATION" << endl;
        return 1;
    }

    for(int i=1; i<argc; ++i) 
        parseArgument(argv[i]);
    
    vilo::BenchmarkNode benchmark;
    benchmark.runFromFolder();

    printf("BenchmarkNode finished.\n");
    return 0;
}

