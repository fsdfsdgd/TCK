// This file is part of VILO: Hybrid Sparse Monocular Visual Odometry
//
// Copyright(c) 2020, Dongting Luo, Dalian University of Technology, Dalian
// Copyright(c) 2020, Smart Robort Group, Dalian University of Technology
//
// This library extends SVO, thus it is highly based on the previous 
// implementation of SVO: https://github.com/uzh-rpg/rpg_svo    
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


/*

This file extends 

P. Bergmann, R. Wang, and D. Cremers, “Online photometric calibration of auto exposure video for realtime visual odometry and SLAM,”
in Proc. of the IEEE Int. Conference on Robotics and Automation (ICRA), 2018.

*/


#include "vilo/photomatric_calibration.h"
#include "vilo/frame.h"
#include "vilo/feature.h"
#include "vilo/config.h"
#include "vilo/point.h"

#include "vilo/vikit/math_utils.h"


namespace vilo{

PhotomatricCalibration::PhotomatricCalibration(int patch_size, int width, int height): m_new_frame_set(false), 
                                                                                       m_width(width), 
                                                                                       m_height(height), 
                                                                                       m_patch_size(patch_size), 
                                                                                       m_max_id(0), 
                                                                                       m_min_id(0), 
                                                                                       m_optimization_block(NULL), 
                                                                                       m_is_optimizing(false)
{
    
}

void PhotomatricCalibration::startThread()
{
   
}

void PhotomatricCalibration::Run()
{
    
}


void PhotomatricCalibration::addFrame(FramePtr frame, int step)
{
    
}


void PhotomatricCalibration::backEndOptimazation()
{
    
}

void PhotomatricCalibration::memoryRelease()
{

}


double PhotomatricCalibration::rapidExposureEstimation(FramePtr curFrame, bool show_debug_prints)
{
    return  0;
}

double PhotomatricCalibration::rapidExposureEstimationNonlinear(FramePtr cur_frame, bool show_debug_prints)
{

    return 0;
}

// [TODO] rotate patch
void PhotomatricCalibration::bulidRadianceFeature(Feature*& ft)
{

}

// [TODO] rotate patch
void PhotomatricCalibration::bulidOutputFeature(Feature*& ft)
{

}

// [TODO] check rotate
void PhotomatricCalibration::getRadianceOutput(Feature*& ft)
{
    
}

//[TODO] 注意：这里是奇数size,change from <= to  <
std::vector<double> 
PhotomatricCalibration::bilinearInterpolateImagePatch(const cv::Mat& image,double x,double y,int patch_size)
{
    std::vector<double> result;
    return result;
}


double PhotomatricCalibration::bilinearInterpolateImage(const cv::Mat& image,double x,double y)
{
    return 0;
}

bool PhotomatricCalibration::extractOptimizationBlock(bool show_debug_prints)
{
    return true;
}

int PhotomatricCalibration::findKeyFrameID(Frame* frame)
{
    return 0;
}

double PhotomatricCalibration::getNormalizedRadius(Vector2d uv_level0)
{
    double x = uv_level0[0];
    double y = uv_level0[1];
    
    assert(x<m_width && y<m_height);

    double x_norm = x - m_width_2;
    double y_norm = y - m_height_2;
    
    double radius = sqrt(x_norm*x_norm + y_norm*y_norm);
    radius /= m_max_radius;
    assert(radius < 1.000000001);

    return radius;
}

// [TODO] direct compute hessian to save time  这里存在eigen矩阵问题
double PhotomatricCalibration::evfOptimization(bool show_debug_prints)
{
    return 0;
}


void PhotomatricCalibration::jacobianRowFillIn(
    cv::Mat& Jacobian, double vignette_value, double r2, double r4, double r6,
    double exposure_time, double radius, double radiance, int id_in_keyframe, int id_in_Jacobian)
{

}


void PhotomatricCalibration::radianceFullOptimization(bool show_debug_prints)
{

}

double PhotomatricCalibration::getTotalResidual(double& total_error, double& avg_error, float huber_th, float outlier_th)
{
    return 0;
}

float PhotomatricCalibration::getMedianError()
{
    return 0;
}

double PhotomatricCalibration::getTotalPointResidual(OptimizedPoint& p, int patch_id)
{
    return 0;
}

double PhotomatricCalibration::getResidualValue(double output, double radiance, double vignette, double exposure)
{
    return 0;
}

double PhotomatricCalibration::applyVignetting(double radius)
{
    return 0;
}

double PhotomatricCalibration::applyVignetting(Vector2d uv)
{
    return applyVignetting(getNormalizedRadius(uv));
}

double PhotomatricCalibration::applyResponse(double evl)
{
    return 0;
}

double PhotomatricCalibration::evaluateGrossbergBaseFunction(int base_function_index, bool is_derivative, double x)
{
    return 0;
}

void PhotomatricCalibration::readResponse(double& c1, double& c2, double& c3, double& c4)
{

}

void PhotomatricCalibration::readVignette(double& v1, double& v2, double& v3)
{

}

void PhotomatricCalibration::setResponse(double c1, double c2, double c3, double c4)
{

}


void PhotomatricCalibration::setVignette(double v1, double v2, double v3)
{

}


void PhotomatricCalibration::setInverseResponseRaw()
{
    
}

void PhotomatricCalibration::readInverseResponseRaw(double* inv)
{
    std::unique_lock<mutex> lock(m_mutex_inverse_response);
    std::memcpy(inv, m_inverse_response, 256*sizeof(double));
}

void PhotomatricCalibration::photometricallyCorrectImage(cv::Mat& corrected_frame)
{
    
}

double PhotomatricCalibration::removeResponse(int output)
{
    return m_inverse_response[output];
}

// [TODO] improve 
void PhotomatricCalibration::setInverseVignette()
{

}

cv::Mat PhotomatricCalibration::readInverseVignette()
{
    std::unique_lock<mutex> lock(m_mutex_inverse_vignette);
    return m_inverse_vignette_image;
}

void PhotomatricCalibration::visualize()
{

}

double PhotomatricCalibration::determineGammaFixResponseAt(double*inverse_response,int x,double y)
{
    return 0;
}

void PhotomatricCalibration::responseInitialize(const cv::Mat& image)
{
    
}



/******************************  windowed  ******************************/
void PhotomatricCalibration::modeConversion()
{
    
}

void PhotomatricCalibration::marginalizeFrame()
{
    
}

double PhotomatricCalibration::applyVignettingZero(double radius)
{
    return 0;
}

double PhotomatricCalibration::applyResponseZero(double evl)
{
    return 0;
}

void PhotomatricCalibration::JacobianZero(Eigen::Matrix<double,1,8>& Jacobian, 
                                       double vignette_value_zero, 
                                       double r2, 
                                       double r4, 
                                       double r6, 
                                       double exposure_time, 
                                       double radiance)
{
   
}

void PhotomatricCalibration::JacobianZeroCV(cv::Mat& Jacobian, 
                                             double vignette_value_zero, 
                                             double r2, 
                                             double r4, 
                                             double r6, 
                                             double exposure_time, 
                                             double radiance,
                                             int id_in_keyframe, 
                                             int id_in_Jacobian)
{

}

double PhotomatricCalibration::getResidualZero(
    double output, double radiance, double vignette_zero, double exposure)
{
    return 0;
}

double PhotomatricCalibration::calcMEnergy()
{
    return 0;
}


void PhotomatricCalibration::saveResult(std::string dir, int sequence_num)
{
   
}

} //namespace vilo