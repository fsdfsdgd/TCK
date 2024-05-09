// This file is part of TCK-VILO:A Tightly-Coupled and Keyframe-Based Visual-Inertial-Lidar Odometry System for Mobile Robots With Adaptive Sensor Reliability Evaluation
//
// Copyright(c) 2024, Jun Yin
//
// This library is highly based on the previous 
// implementation of HSO(https://github.com/luodongting/HSO),   
// A-LOAM (https://github.com/HKUST-Aerial-Robotics/A-LOAM), and
// ORB-SLAM3(https://github.com/UZ-SLAMLab/ORB_SLAM3)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sys/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vilo {


class FileReader
{
public:

    FileReader(std::string image_folder, cv::Size new_size, std::string time_folder="None");
    FileReader(std::string image_folder, std::string time_folder="None");
    FileReader();

    int readEurocFilesTimes(std::string image_folder, 
                            std::vector<std::string>& imagefolder_vec,
                            std::vector<double>& time_vec);
    int readEurocImu(std::string imu_folder, 
                     std::vector<Eigen::Vector3d>& imu_acc_vec,
                     std::vector<Eigen::Vector3d>& imu_gyro_vec,
                     std::vector<double>& time_vec);

    int readFileNamesTimes(std::string file_folder, 
                           std::vector<std::string>& file_name_vec,
                           std::vector<double>& time_vec,
                           int type);

    int readFileNames(std::string file_folder, 
                      std::vector<std::string>& file_name_vec,
                      int type);

    int readFileTimes(std::string file_folder, 
                      std::vector<double>& time_vec);
   
    int readImu(std::string imu_folder, 
                         std::vector<Eigen::Vector3d>& imu_acc_vec,
                         std::vector<Eigen::Vector3d>& imu_gyro_vec,
                         std::vector<double>& time_vec);
    
    int readImuTXT(std::string imu_folder, 
                         std::vector<Eigen::Vector3d>& imu_acc_vec,
                         std::vector<Eigen::Vector3d>& imu_gyro_vec,
                         std::vector<double>& time_vec);


    cv::Mat readImage(int image_index);
    cv::Mat readImage(int image_index , cv::Size new_size);
    void readPointsFromBIN(int laser_index);
    void readPointsFromPCD(int id, std::vector<double>& pts_ts_vec);
    void readPointsFromTXT(int id, std::vector<double>& pts_ts_vec);
    void readPointsFromBIN(int id, std::vector<double>& pts_ts_vec);
    void readFromTXTSaveToBin(int id);
    std::string readStamp(int image_index);

    int getNumImages() { return (int)m_image_files.size(); }
    int getNumLidars() { return (int)m_lidar_files.size(); }

    int getDir(std::string dir, std::vector<std::string> &files);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLidarPtr(){return cloud_;}

    inline bool stampValid() { return m_stamp_valid; }
    
private:
    
    cv::Size m_img_new_size;

    bool m_stamp_valid;

    std::vector<std::string> m_image_files;
    std::vector<std::string> m_lidar_files;
    std::vector<std::string> m_times;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};
}
