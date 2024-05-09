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

#include <mutex>
#include <opencv2/core/core.hpp>
#include <sophus/se3.h>
#include <pangolin/pangolin.h>

namespace vilo {
class Frame;
class Map;
class FrameHandlerMono; }


namespace vilo {

class Viewer
{
public:
    Viewer(vilo::FrameHandlerMono* vo);
    void run();
    bool CheckFinish();
    void DrawKeyFrames(const bool bDrawKF);
    void DrawMapRegionPoints();
    void DrawMapSeeds();
    void DrawConstraints();

    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

private:
    vilo::FrameHandlerMono* _vo;

    std::mutex mMutexCurrentPose;
    std::vector< Sophus::SE3 > visual_pos_vec_;
    std::vector< Sophus::SE3 > lidar_pos_vec_;
    bool m_need_clear = true;
    Sophus::SE3  _CurrentPoseTwc ;
    int _drawedframeID=0;

    void SetFinish();
    bool mbFinished;
    std::mutex mMutexFinish;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
};
}
