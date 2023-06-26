/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
//
// It is similar to the RealTimeCorrelativeScanMatcher but has a different
// trade-off: Scan matching is faster because more effort is put into the
// precomputation done for a given map. However, this map is immutable after
// construction.
#pragma once

#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>

#include "Eigen/Core"
// #include "cartographer/common/port.h"
// #include "cartographer/mapping/2d/grid_2d.h"
#include "correlative_scan_matcher_2d.h"
#include "msa2d/Map/OccGridMapBase.hpp"
// #include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
// #include "cartographer/sensor/point_cloud.h"

namespace msa2d {
namespace ScanMatcher {

class PrecomputationGrid2D {
 public:
  PrecomputationGrid2D(map::OccGridMapBase* grid_map, const int cell_width);

  // Returns a value between 0 and 255 to represent probabilities between
  // min_score and max_score.
  // 获取栅格值    此时 栅格的值已经从概率 0-1 转换成了 0 - 255 
  int GetValue(const Eigen::Array2i& xy_index) const {
    const Eigen::Array2i local_xy_index = xy_index - offset_;    // offset_ 是地图膨胀后增加的值
    // The static_cast<unsigned> is for performance to check with 2 comparisons
    // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
    // local_xy_index.x() >= wide_limits_.num_x_cells ||
    // local_xy_index.y() >= wide_limits_.num_y_cells
    // instead of using 4 comparisons.
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(map_grid_size_.x()) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(map_grid_size_.y())) {
      return 0;
    }
    const int stride = map_grid_size_.x();
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
  }

  // Maps values from [0, 255] to [min_score, max_score].
  float ToScore(float value) const {
    return min_score_ + value * ((max_score_ - min_score_) / 255.f);
  }

  cv::Mat ToImage() const; 

  const Eigen::Vector2i& GetMapGridSize() const;  

 private:
  uint8_t ComputeCellValue(float probability) const;

  const Eigen::Array2i offset_;
  const Eigen::Vector2i map_grid_size_;    // 地图x方向与y方向的格子数

  const float min_score_;
  const float max_score_;

  // Probabilites mapped to 0 to 255.
  std::vector<uint8_t> cells_;   // 不同分辨率的栅格地图
};

struct FastCorrelativeScanMatcherOptions2D {
    int branch_and_bound_depth_ = 5; 
    int linear_search_window_ = 5;   // xy搜索区域大小    m
    int angular_search_window_ = 30;
    int first_layer_resolution_ = 4;    // 第一层的是4倍的分辨率  
};

class PrecomputationGridStack2D {
public:
  PrecomputationGridStack2D(
      map::OccGridMapBase* grid_map,
      const FastCorrelativeScanMatcherOptions2D& options);

  // 获取指定层的地图
  const PrecomputationGrid2D& Get(int index) {
    return precomputation_grids_[index];
  }

  const SearchParameters::LinearBounds& GetValidGridMapRange() const;  

  int max_depth() const { return precomputation_grids_.size() - 1; }
  const int& first_layer_resolution() const {
    return first_layer_resolution_;
  }
private:
  int first_layer_resolution_;  
  SearchParameters::LinearBounds valid_grid_range_{9999, 0, 9999, 0};  // 有效栅格的范围  
  std::vector<PrecomputationGrid2D> precomputation_grids_;
};

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class FastCorrelativeScanMatcher2D {
 public:
  FastCorrelativeScanMatcher2D(
      map::OccGridMapBase* grid_map,
      const FastCorrelativeScanMatcherOptions2D& options);
  ~FastCorrelativeScanMatcher2D();

  FastCorrelativeScanMatcher2D(const FastCorrelativeScanMatcher2D&) = delete;
  FastCorrelativeScanMatcher2D& operator=(const FastCorrelativeScanMatcher2D&) = delete;

  bool Match(const Pose2d& initial_pose_estimate,
             const sensor::LaserPointCloud& point_cloud, float min_score,
             float& score, Pose2d& pose_estimate) const;

  bool MatchFullSubmap(const sensor::LaserPointCloud& point_cloud, float min_score,
                       float& score, Pose2d& pose_estimate) const;

 private:
  // The actual implementation of the scan matcher, called by Match() and
  // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
  // 'search_parameters'.
  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const Pose2d& initial_pose_estimate,
      const sensor::LaserPointCloud& point_cloud, 
      float min_score, float& score,
      Pose2d& pose_estimate) const;

  std::vector<Candidate2D> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan2D>& discrete_scans,
      const SearchParameters& search_parameters) const;

  std::vector<Candidate2D> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;

  void ScoreCandidates(const PrecomputationGrid2D& precomputation_grid,
                       const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* const candidates) const;

  Candidate2D BranchAndBound(const std::vector<DiscreteScan2D>& discrete_scans,
                             const SearchParameters& search_parameters,
                             const std::vector<Candidate2D>& candidates,
                             int candidate_depth, float min_score) const;

  const FastCorrelativeScanMatcherOptions2D options_;
  const float map_resolution_;
  std::unique_ptr<PrecomputationGridStack2D> precomputation_grid_stack_;
  const map::OccGridMapBase* grid_map_;   
};

}  // namespace 
}  // namespace 
