// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "vilo/vikit/so3_functions.h"
#include <iostream>

namespace vilo {

using namespace Eigen;

Eigen::Matrix3d normalizeRotation(const Eigen::Matrix3d &R)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R,Eigen::ComputeFullU | Eigen::ComputeFullV);
    // std::cout<<"svd.matrixU() : "<<std::endl<<svd.matrixU()<<std::endl;
    // std::cout<<"svd.matrixV().transpose() : "<<std::endl<<svd.matrixV().transpose()<<std::endl;
    // cout<<"R.det"<<R.determinant()<<endl;
    // cout<<"U.det"<<svd.matrixU().determinant()<<endl;
    // cout<<"V.det"<<svd.matrixU().determinant()<<endl;
    return svd.matrixU()*svd.matrixV().transpose();
}

Eigen::Matrix3d expSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W +0.5*W*W;
        return normalizeRotation(res);
    }
    else
    {
        Eigen::Matrix3d res =Eigen::Matrix3d::Identity() + W*sin(d)/d + W*W*(1.0-cos(d))/d2;
        return normalizeRotation(res);
    }
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d &w)
{
    return expSO3(w[0],w[1],w[2]);
}



Eigen::Vector3d logSO3(const Eigen::Matrix3d &R)
{
    const double tr = R(0,0)+R(1,1)+R(2,2);
    Eigen::Vector3d w;
    w << (R(2,1)-R(1,2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
    const double costheta = (tr-1.0)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const double theta = acos(costheta);
    const double s = sin(theta);
    if(fabs(s)<1e-5)
        return w;
    else
        return theta*w/s;
}

Eigen::Matrix3d inverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }  
    else
    {
        return Eigen::Matrix3d::Identity() + W/2 + W*W*(1.0/d2 - (1.0+cos(d))/(2.0*d*sin(d)));
    }
        
}
Eigen::Matrix3d inverseRightJacobianSO3(const Eigen::Vector3d &v)
{
    return inverseRightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d rightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        return Eigen::Matrix3d::Identity() - W*(1.0-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}


Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d &v)
{
    return rightJacobianSO3(v[0],v[1],v[2]);
}


Eigen::Matrix3d skew(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d W;
    W << 0.0, -w[2], w[1],w[2], 0.0, -w[0],-w[1],  w[0], 0.0;
    return W;
}

} // end namespace 
