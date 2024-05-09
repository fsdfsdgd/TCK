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

#ifndef VILO_REPROJECTION_H_
#define VILO_REPROJECTION_H_

#include <vilo/global.h>
#include <vilo/matcher.h>
#include "vilo/camera.h"


namespace vilo {

class Map;
class Point;
class DepthFilter;
struct Seed;

class Reprojector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options {
    size_t max_n_kfs; 
    bool find_match_direct;
    bool reproject_unconverged_seeds;
    float reproject_seed_thresh;
    Options()
    : max_n_kfs(10),
    find_match_direct(true),
    reproject_unconverged_seeds(true),
    reproject_seed_thresh(86)
    {}
  } options_;

  size_t n_matches_;
  size_t n_trials_;
  size_t n_seeds_;
  size_t n_filters_;

  FramePtr lastFrame_;

  Reprojector(vilo::AbstractCamera* cam, Map* map);
  Reprojector(Map* map);

  ~Reprojector();

  
  void initializeGrid(vilo::AbstractCamera* cam);
  int caculateGridSize(const int wight, const int height, const int N);
  void reprojectMap( FramePtr frame);

  DepthFilter* depth_filter_;

private:

  struct Candidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Point* pt;   
    Vector2d px;   

    Candidate(Point* pt, Vector2d& px) :  pt(pt), px(px) 
    {}
  };
  typedef std::list<Candidate > Cell;
  typedef std::vector<Cell*> CandidateGrid;

  struct SeedCandidate{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Seed& seed;
    Vector2d px;
    list< Seed, aligned_allocator<Seed> >::iterator index;
    SeedCandidate(Seed& _seed, Vector2d _uv, 
      list< Seed, aligned_allocator<Seed> >::iterator _i):seed(_seed), px(_uv), index(_i)
    {}
  };
  typedef std::list<SeedCandidate> Sell;
  typedef std::vector<Sell*> SeedGrid;

  struct Grid
  {
    CandidateGrid cells;
    SeedGrid seeds;
    vector<int> cell_order;
    int cell_size;
    int grid_n_cols;
    int grid_n_rows;

    int cell_size_w;
    int cell_size_h;
  };

  Grid grid_;
  Matcher matcher_;
  Map* map_;

  static bool pointQualityComparator(Candidate& lhs, Candidate& rhs);
  static bool seedComparator(SeedCandidate& lhs, SeedCandidate& rhs);

  void resetGrid();
  bool reprojectCell(Cell& cell, FramePtr frame, bool is_2nd = false, bool is_3rd = false);
  bool reprojectorSeeds(Sell& sell, FramePtr frame);

  bool checkPoint(FramePtr frame, Point* point, vector< pair<Vector2d, Point*> >& cells);

  bool checkSeed(FramePtr frame, Seed& seed, list< Seed, aligned_allocator<Seed> >::iterator index);

  void reprojectCellAll(vector< pair<Vector2d, Point*> >& cell, FramePtr frame);

  size_t sum_seed_;
  size_t sum_temp_;

  size_t nFeatures_;
};

} // namespace vilo

#endif // VILO_REPROJECTION_H_
