#pragma once

#include <vector>
#include "Eigen/Core"
// #include "cartographer/common/lua_parameter_dictionary.h"
// #include "cartographer/mapping/2d/map_limits.h"
// #include "cartographer/mapping/2d/xy_index.h"
// #include "cartographer/sensor/point_cloud.h"
#include "robo2d/Sensor/point_cloud.hpp"
#include "robo2d/Map/GridMapBase.hpp"

namespace robo2d {
namespace ScanMatcher {

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

// Describes the search space.
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::LaserPointCloud& point_cloud, double resolution, 
                   const int& expansion_coeff, const int& map_depth);

  // Tightens the search window as much as possible.
  bool ShrinkToFit(const LinearBounds& map_bounds,
                   const map::GridMapBase* map, const Pose2d& initial_pose_estimate);

  int angular_search_step_num_one_side_;            // 一边的角度搜索步数 
  double angular_perturbation_step_;    // 角度扰动量
  double resolution_;
  int rotated_scans_num_;     // 旋转后的点云集合的个数
  std::vector<LinearBounds> linear_bounds_;  // Per rotated scans.
  std::vector<int> angular_perturbation_steps_;
};

// Generates a collection of rotated scans.
std::vector<sensor::LaserPointCloud> GenerateRotatedScans(
    const sensor::LaserPointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan2D> DiscretizeScans(
    const map::GridMapBase* map, const std::vector<sensor::LaserPointCloud>& scans,
    const Pose2d& initial_pose_estimate);

// A possible solution.
struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),   // 旋转点云序号
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(x_index_offset * search_parameters.resolution_),
        y(y_index_offset * search_parameters.resolution_),
        orientation((scan_index - search_parameters.angular_search_step_num_one_side_) *
                    search_parameters.angular_perturbation_step_) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate2D relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;
  // 用于std::max比较  
  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

}  // namespace 
}  // namespace 
