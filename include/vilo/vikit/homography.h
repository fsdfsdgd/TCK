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

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/SVD>

#include "vilo/vikit/math_utils.h"

namespace vilo {

using namespace Eigen;
using namespace std;

struct HomographyDecomposition
{
  Vector3d t;
  Matrix3d R;
  double   d;
  Vector3d n;

  // Resolved  Composition
  Sophus::SE3 T; //!< second from first
  int score;
};

class Homography
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Homography            (const vector<Vector2d, aligned_allocator<Vector2d> >& _fts1,
                         const vector<Vector2d, aligned_allocator<Vector2d> >& _fts2,
                         double _error_multiplier2,
                         double _thresh_in_px);

  void
  calcFromPlaneParams   (const Vector3d & normal,
                         const Vector3d & point_on_plane);

  void
  calcFromMatches       ();

  size_t
  computeMatchesInliers ();

  bool
  computeSE3fromMatches ();

  bool
  decompose             ();

  void
  findBestDecomposition ();

  double thresh;
  double error_multiplier2;
  const vector<Vector2d, aligned_allocator<Vector2d> >& fts_c1; //!< Features on first image on unit plane
  const vector<Vector2d, aligned_allocator<Vector2d> >& fts_c2; //!< Features on second image on unit plane
  vector<bool> inliers;
  SE3 T_c2_from_c1;             //!< Relative translation and rotation of two images
  Matrix3d H_c2_from_c1;                   //!< Homography
  vector<HomographyDecomposition> decompositions;
};


} /* end namespace vk */

