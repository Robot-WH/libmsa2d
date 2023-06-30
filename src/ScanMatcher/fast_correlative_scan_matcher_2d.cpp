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
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "glog/logging.h"
#include "msa2d/ScanMatcher/fast_correlative_scan_matcher_2d.h"
#include "msa2d/common/tic_toc.h"

namespace msa2d {
namespace ScanMatcher {
namespace {

/************** SlidingWindowMaximum **************/

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
// 滑动窗口算法
class SlidingWindowMaximum {
 public:
    // 添加值, 会将小于填入值的其他值删掉, 再将这个值放到最后
    void AddValue(const float& value) {
        while (!non_ascending_maxima_.empty() &&
              value > non_ascending_maxima_.back()) {
          non_ascending_maxima_.pop_back();
        }
        non_ascending_maxima_.push_back(value);
    }

  // 删除值, 如果第一个值等于要删除的这个值, 则将这个值删掉
  void RemoveValue(const float& value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid_map.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  // 获取最大值, 因为是按照顺序存储的, 第一个值是最大的
  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid_map.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};
}  // namespace

/************** PrecomputationGrid2D **************/

// 构造不同分辨率的地图
PrecomputationGrid2D::PrecomputationGrid2D(
    map::OccGridMapBase* grid_map, const int cell_width)
    : offset_(-cell_width / 2, -cell_width / 2),
    // : offset_(-cell_width + 1, -cell_width + 1),
      map_grid_size_(grid_map->getGridMapBase().getGridSizeX() + cell_width - 1,
                   grid_map->getGridMapBase().getGridSizeY()+ cell_width - 1),
      min_score_(0), // 0.1 min_score_
      max_score_(1), // 0.9 max_score_
      // cell_即栅格地图数据
      cells_(map_grid_size_.x() * map_grid_size_.y()) {
  // std::cout << "offset_: " << offset_.transpose() << std::endl;
  // std::cout << "PrecomputationGrid2D build! cell_width: " << cell_width
  //   << ", map_grid_size_: " << map_grid_size_.transpose() 
  //   << ",cells_ size: " << cells_.size() << std::endl;
  CHECK_GE(cell_width, 1);    // >=1 
  Eigen::Vector2i origin_map_grid_size = grid_map->getGridMapBase().getMapGridSize(); 
  const int stride = map_grid_size_.x();
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + cell_width.

    std::vector<float> intermediate;
    intermediate.resize(map_grid_size_.x() * origin_map_grid_size.y());   // 临时的地图数据
  
  int grid_value = 0;  
  // 先对x方向进行分辨率膨胀
  // 对每一行从左到右横着做一次滑窗, 将滑窗后的地图放在intermediate(临时数据)中
  for (int y = 0; y != origin_map_grid_size.y(); ++y) {
    SlidingWindowMaximum current_values;

    if (grid_map->isUnknow(0, y)) {
      current_values.AddValue(0.5f);  // 添加占据的概率 
    } else if (grid_map->isFree(0, y)) {
      current_values.AddValue(0.01f);  // 添加占据的概率 
    } else {
      current_values.AddValue(0.99f);  // 添加占据的概率 
    }
    
    // Step: 1 滑动窗口在x方向开始划入地图, 所以只进行 填入值
    for (int x = -cell_width + 1; x != 0; ++x) {
      intermediate[x + cell_width - 1 + y * stride] = current_values.GetMaximum();
      if (x + cell_width < origin_map_grid_size.x()) {

        if (grid_map->isUnknow(x + cell_width, y)) {
          current_values.AddValue(0.5f);
        } else if (grid_map->isFree(x + cell_width, y)) {
          current_values.AddValue(0.01f);
        } else {
          current_values.AddValue(0.99f);
        }
      }
    }

    // Step: 2 滑动窗口已经完全在地图里了, 滑窗进行一入一出的操作
    for (int x = 0; x < origin_map_grid_size.x() - cell_width; ++x) {
      intermediate[x + cell_width - 1 + y * stride] = current_values.GetMaximum();

      if (grid_map->isUnknow(x, y)) {
        current_values.RemoveValue(0.5f);
      } else if (grid_map->isFree(x, y)) {
        current_values.RemoveValue(0.01f);
      } else {
        current_values.RemoveValue(0.99f);
      }

      if (grid_map->isUnknow(x + cell_width, y)) {
        current_values.AddValue(0.5f);
      } else if (grid_map->isFree(x + cell_width, y)) {
        current_values.AddValue(0.01f);
      } else {
        current_values.AddValue(0.99f);
      }
    }

    // Step: 3 滑动窗口正在划出, 一次减少一个值, 所以intermediate的宽度比grid多 cell_width-1
    for (int x = std::max(origin_map_grid_size.x() - cell_width, 0);
         x != origin_map_grid_size.x(); ++x) {
      intermediate[x + cell_width - 1 + y * stride] = current_values.GetMaximum();

      if (grid_map->isUnknow(x, y)) {
        current_values.RemoveValue(0.5f);
      } else if (grid_map->isFree(x, y)) {
        current_values.RemoveValue(0.01f);
      } else {
        current_values.RemoveValue(0.99f);
      }
    }
    // 理论上, 滑窗走完地图的一行之后应该是空的, 经过 只入, 一出一入, 只出, 3个步骤
    current_values.CheckIsEmpty();
  }

  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.

  // 根据intermediate的值, 对每一列从下到上竖着再做一次滑窗, 这个才是真正的地图cells_
  for (int x = 0; x != map_grid_size_.x(); ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);

    for (int y = -cell_width + 1; y != 0; ++y) {
      cells_[x + (y + cell_width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());   // 会转换成0-255
      if (y + cell_width < origin_map_grid_size.y()) {
        current_values.AddValue(intermediate[x + (y + cell_width) * stride]);
      }
    }

    for (int y = 0; y < origin_map_grid_size.y() - cell_width; ++y) {
      cells_[x + (y + cell_width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + cell_width) * stride]);
    }

    for (int y = std::max(origin_map_grid_size.y() - cell_width, 0);
         y != origin_map_grid_size.y(); ++y) {
      cells_[x + (y + cell_width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }

    current_values.CheckIsEmpty();
  }
}

// 将概率[0.1, 0.9]转成[0, 255]之间的值
uint8_t PrecomputationGrid2D::ComputeCellValue(const float probability) const {
  const int cell_value = std::round(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

cv::Mat PrecomputationGrid2D::ToImage() const {
  // 构造opencv mat 
  cv::Mat map_img(map_grid_size_.y(), map_grid_size_.x(), CV_8UC1, cv::Scalar(255));
  // 转为灰度
  for (int row = 0; row < map_grid_size_.y(); row++) {
    for (int col = 0; col < map_grid_size_.x(); col++) {
        map_img.at<uint8_t>(map_grid_size_.y() - 1 - row, col) = 
          cells_[col + map_grid_size_.x() * row];
    }
  }
  return map_img; 
}

const Eigen::Vector2i& PrecomputationGrid2D::GetMapGridSize() const {
  return map_grid_size_;  
}

// 构造多分辨率地图
PrecomputationGridStack2D::PrecomputationGridStack2D(
      map::OccGridMapBase* grid_map,
      const FastCorrelativeScanMatcherOptions2D& options) {
    CHECK_GE(options.branch_and_bound_depth_, 1);
    // 1 3 5 9 ...
    // param: branch_and_bound_depth 默认为7, 确定 最大的分辨率, 也就是64个栅格合成一个格子
    min_resolution_ = options.first_layer_expansion_length_ * 2 + 1;
    max_resolution_ = 1 + 2 * options.first_layer_expansion_length_ 
      * (1 << (options.branch_and_bound_depth_ - 1));
    // std::cout << "min_resolution_: " << min_resolution_ << std::endl;
    // std::cout << "max_resolution_: " << max_resolution_ << std::endl;
    precomputation_grids_.reserve(options.branch_and_bound_depth_);
    // time::TicToc tt;
    // static float avg_time = 0; 
    // static int N = 0; 
    // 分辨率逐渐变大, i = 0时就是默认分辨率0.05, i=6时, width=64,也就是64个格子合成一个值
    for (int i = 0; i != options.branch_and_bound_depth_; ++i) {
      // for (int i = 0; i < 2; ++i) {
      // time::TicToc tt;
      const int width = 1 + 2 * options.first_layer_expansion_length_ * (1 << i);
      // std::cout << "width: " << width << std::endl;
      // 构造不同分辨率的地图 PrecomputationGrid2D
      precomputation_grids_.emplace_back(grid_map, width);
      // 如果是原始地图，计算有效栅格覆盖的范围
      // if (i == 0) {
      //   // time::TicToc tt;
      //   const Eigen::Vector2i& map_grid_size = precomputation_grids_[0].GetMapGridSize(); 

      //   for (int row = 0; row < map_grid_size.y(); ++row) {
      //     for (int col = 0; col < map_grid_size.x(); ++col) {
      //       if (precomputation_grids_[0].GetValue(Eigen::Array2i(col, row)) != 128) {
      //         // std::cout << "value: " << precomputation_grids_[0].GetValue(Eigen::Array2i(col, row)) << std::endl;
      //         if (col < valid_grid_range_.min_x) {
      //           valid_grid_range_.min_x = col;
      //         } 
      //         if (col > valid_grid_range_.max_x) {
      //           valid_grid_range_.max_x = col;
      //         }

      //         if (row < valid_grid_range_.min_y) {
      //           valid_grid_range_.min_y = row;
      //         } 
      //         if (row > valid_grid_range_.max_y) {
      //           valid_grid_range_.max_y = row;
      //         }
      //       }
      //     }
      //   }
      //   // tt.toc("map range: ");
      //   // std::cout << "valid_grid_range_.min_x: " << valid_grid_range_.min_x << std::endl;
      //   // std::cout << "valid_grid_range_.max_x: " << valid_grid_range_.max_x << std::endl;
      //   // std::cout << "valid_grid_range_.min_y: " << valid_grid_range_.min_y << std::endl;
      //   // std::cout << "valid_grid_range_.max_y: " << valid_grid_range_.max_y << std::endl;
      // }
      // tt.toc("PrecomputationGridStack2D");
    }
    // float t = tt.toc("PrecomputationGridStack2D");
    // ++N;
    // avg_time += (t - avg_time) / N;  // 估计样本均值
    // std::cout << "avg_time: " << avg_time << std::endl;
}

const SearchParameters::LinearBounds& 
PrecomputationGridStack2D::GetValidGridMapRange() const {
  return valid_grid_range_;   
}

/************** FastCorrelativeScanMatcher2D **************/

// 构造函数
FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    map::OccGridMapBase* grid_map,
    const FastCorrelativeScanMatcherOptions2D& options)
    : options_(options),
      grid_map_(grid_map),
      map_resolution_(grid_map->getGridMapBase().getGridResolution()),
      // 多分辨率地图的构建
      precomputation_grid_stack_(
          std::make_unique<PrecomputationGridStack2D>(grid_map, options)) {
}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

/**
 * @brief 进行局部搜索窗口的约束计算(对局部子图进行回环检测)
 * 
 * @param[in] initial_pose_estimate 先验位姿
 * @param[in] point_cloud 原点位于local坐标系原点处的点云
 * @param[in] min_score 最小阈值, 低于这个分数会返回失败
 * @param[out] score 匹配后的得分
 * @param[out] pose_estimate 匹配后得到的位姿
 * @return true 匹配成功, 反之匹配失败
 */
bool FastCorrelativeScanMatcher2D::Match(
    const Pose2d& initial_pose_estimate,
    const sensor::LaserPointCloud& point_cloud, const float min_score, float& score,
    Pose2d& pose_estimate) const {
  // param: linear_search_window angular_search_window 
  const SearchParameters search_parameters(options_.linear_search_window_, // 7
                                           options_.angular_search_window_, // 30
                                           point_cloud, map_resolution_, 
                                           2 * options_.first_layer_expansion_length_ + 1,
                                           options_.branch_and_bound_depth_);
  // 带入搜索参数进行搜粟，搜索中心为先验位姿
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

/**
 * @brief 进行全局搜索窗口的约束计算(对整体子图进行回环检测)
 * 
 * @param[in] point_cloud 原点位于local坐标系原点处的点云
 * @param[in] min_score 最小阈值, 低于这个分数会返回失败
 * @param[out] score 匹配后的得分
 * @param[out] pose_estimate 匹配后得到的位姿
 * @return true 匹配成功, 反之匹配失败
 */
bool FastCorrelativeScanMatcher2D::MatchFullSubmap(
      const sensor::LaserPointCloud& point_cloud, float min_score, float& score,
      Pose2d& pose_estimate) const {
  // 将搜索窗口设置成 xy范围是1e6米, 角度范围是M_PI
  const SearchParameters search_parameters(
      1e6 * map_resolution_,  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, 
      map_resolution_, 
      2 * options_.first_layer_expansion_length_ + 1,
      options_.branch_and_bound_depth_);
  // 计算搜索窗口的中点 把这个中点作为搜索的起点
  const Eigen::Vector2i&  map_world_size = grid_map_->getGridMapBase().getMapGridSize();  
  const Pose2d center = Pose2d(map_world_size.x() / 2, map_world_size.y() / 2, 0);
  
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

// 进行基于分支定界算法的粗匹配
bool FastCorrelativeScanMatcher2D::MatchWithSearchParameters(
      SearchParameters search_parameters,
      const Pose2d& initial_pose_estimate,
      const sensor::LaserPointCloud& point_cloud, float min_score, float& score,
      Pose2d& pose_estimate) const {
    
    time::TicToc tt; 
    // Step: 将原点处的点云先旋转到预测的方向上
    const sensor::LaserPointCloud rotated_point_cloud = sensor::TransformPointCloud(
        point_cloud, Pose2d(0, 0, initial_pose_estimate.yaw()));
    // Step: 生成按照不同角度旋转后的点云集合
    const std::vector<sensor::LaserPointCloud> rotated_scans =
        GenerateRotatedScans(rotated_point_cloud, search_parameters);
    // Step: 将旋转的点云集合平移移动到搜索起点，并获取每个点的地图栅格坐标
    const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
        &grid_map_->getGridMapBase(), rotated_scans, initial_pose_estimate);
    // Step: 获取地图的有效栅格范围
    // 缩小搜索窗口的大小 同时检查地图范围是否足够大   
    if (!search_parameters.ShrinkToFit(precomputation_grid_stack_->GetValidGridMapRange(), 
                                                                      &grid_map_->getGridMapBase(), 
                                                                      initial_pose_estimate)) {
      return false;  
    }
    tt.tic();  
    // 计算最低分辨率中的所有的候选解的得分，并从大到小进行排序
    // 这里就是分支定界法中确定分支上界的环节
    const std::vector<Candidate2D> lowest_resolution_candidates =
        ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
    tt.toc("ComputeLowestResolutionCandidates ");
    tt.tic();  
    // Step: 进行基于分支定界算法的搜索, 获取最优解
    const Candidate2D best_candidate = BranchAndBound(
        discrete_scans, search_parameters, lowest_resolution_candidates,
        precomputation_grid_stack_->max_depth(), precomputation_grid_stack_->max_resolution() / 2,
        precomputation_grid_stack_->max_resolution() / precomputation_grid_stack_->min_resolution(),
        min_score); // param: max_depth

    tt.toc("BranchAndBound ");
    std::cout << "best_candidate.score: " << best_candidate.score << std::endl;
    // std::cout << "best_candidate.x: " << best_candidate.x << std::endl;
    // std::cout << "best_candidate.y: " << best_candidate.y << std::endl;
    // std::cout << "best_candidate.orientation: " << best_candidate.orientation << std::endl;
    // 检查最优解的值, 如果大于指定阈值min_score就认为匹配成功,否则认为不匹配返回失败
    if (best_candidate.score > min_score) {
        score = best_candidate.score;
        // Step: 根据计算出的偏移量对位姿进行校准
        pose_estimate = Pose2d(
            initial_pose_estimate.x() + best_candidate.x_index_offset,
            initial_pose_estimate.y() + best_candidate.y_index_offset,
            initial_pose_estimate.yaw() + best_candidate.orientation);
        return true;
    }
    return false;
}

// 生成最低分辨率层(栅格最粗)上的所有候选解, 并进行打分与排序
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  // 生成最低分辨率层(栅格最粗)上的所有候选解
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  // 计算每个候选解的得分, 按照匹配得分从大到小排序, 返回排列好的candidates 
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),  // 最粗分辨率  
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

/**
 * @brief 生成最低分辨率层(栅格最粗)上的所有候选解
 *                检查：错误可能性低
 * @param search_parameters 
 * @return std::vector<Candidate2D> 
 */
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  // 分辨率    
  int linear_step_size = precomputation_grid_stack_->max_resolution();   // 1 << x, 即 2^(x-1)
  const int& min_step_size = precomputation_grid_stack_->min_resolution();  
  int angle_step = linear_step_size / min_step_size;
  // std::cout << "angle_step: " << angle_step << std::endl;
  int num_candidates = 0;
  linear_step_size /= 2;
  // 遍历旋转后的每个点云     num_scans 即旋转点云的数量
  for (int scan_index = 0; scan_index < search_parameters.rotated_scans_num_ + angle_step - 1;
       scan_index += angle_step) {

    if (scan_index >= search_parameters.rotated_scans_num_) {
      scan_index = search_parameters.rotated_scans_num_ - 1;  
    }
    // X方向候选解的个数
    // max_x 表示从起点向x增加方向搜索的size，min_x 表示从起点向x减少方向搜索的size
    // 例如  max_x = 1，min_x = -1， 则  候选解一共有  | <--- | ---- >|    3个 
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds_[scan_index].max_x -
         search_parameters.linear_bounds_[scan_index].min_x + linear_step_size) /
        linear_step_size;

    // Y方向候选解的个数
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds_[scan_index].max_y -
         search_parameters.linear_bounds_[scan_index].min_y + linear_step_size) /
        linear_step_size;

    // num_candidates 为最低分辨率这一层中所有候选解的总个数
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }

  // std::cout << "num_candidates: " << num_candidates << std::endl;

  // 将所有候选解保存起来, 候选解的结构为（角度的索引, x偏移量, y偏移量, 搜索参数）
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);

  for (int scan_index = 0; scan_index != search_parameters.rotated_scans_num_ + angle_step - 1;
       scan_index += angle_step) {

    if (scan_index >= search_parameters.rotated_scans_num_) {
      scan_index = search_parameters.rotated_scans_num_ - 1;  
    }

    for (int x_index_offset = search_parameters.linear_bounds_[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds_[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds_[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds_[scan_index].max_y;
           y_index_offset += linear_step_size) {
        // 生成候选解, 存的是候选解与原始点云原点坐标间的偏移量
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);   // scan_index 反映了旋转角度  
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

/**
 * @brief 对所有的候选解进行评分并进行降序排序
 *                  每一个候选解 就是一个枝，这里求解但得分就是这个枝的上界  
 * 
 * @param precomputation_grid 
 * @param discrete_scans 
 * @param search_parameters 
 * @param candidates 
 */
void FastCorrelativeScanMatcher2D::ScoreCandidates(
      const PrecomputationGrid2D& precomputation_grid,
      const std::vector<DiscreteScan2D>& discrete_scans,
      const SearchParameters& search_parameters,
      std::vector<Candidate2D>* const candidates) const {
    // 遍历所有的候选解, 对每个候选解进行打分
    /**
     * @todo 可以并行加速  
     */
    for (Candidate2D& candidate : *candidates) {
      int sum = 0;
      // xy_index 为这帧旋转后的点云上的每个点对应在地图上的栅格坐标
      for (const Eigen::Array2i& xy_index :
          discrete_scans[candidate.scan_index]) {       // discrete_scans[candidate.scan_index] 获得了该旋转角度下的点云
        // 旋转后的点云的每个点的坐标加上这个可行解的X与Y的偏置, 即将点云进行平移
        const Eigen::Array2i proposed_xy_index(
            xy_index.x() + candidate.x_index_offset,
            xy_index.y() + candidate.y_index_offset);

        int value = precomputation_grid.GetValue(proposed_xy_index);
        // if (value > 128)  value = 255;
        // else if (value < 128) value = 0;
        // 对平移后的点云的每个点 获取在precomputation_grid上对应的栅格值
        // sum += precomputation_grid.GetValue(proposed_xy_index);   // 0 - 255 
        sum += value;   // 0 - 255 
      }
      // 栅格值的和除以这个点云中点的个数, 作为这个候选解在这个 precomputation_grid 上的得分
      candidate.score = precomputation_grid.ToScore(
          sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
      // std::cout << "candidate.score: " << candidate.score << ", x_index_offset: " << candidate.x_index_offset
      //   << ",candidate.y_index_offset: " << candidate.y_index_offset  << ", scan_index: " << candidate.scan_index
      //   << std::endl;
    }
    // 根据候选解的score, 对所有候选解进行降序排列
    std::sort(candidates->begin(), candidates->end(),
              std::greater<Candidate2D>());
  //   std::cout << "candidate.best score: " << (*candidates)[0].score << ", x_index_offset: " << (*candidates)[0].x_index_offset
  // << ",candidate.y_index_offset: " << (*candidates)[0].y_index_offset  << ", scan_index: " << (*candidates)[0].scan_index
  // << std::endl;
}

/**
 * @brief 基于多分辨率地图的分支定界搜索算法
 * 
 * @param[in] discrete_scans 多个点云的每个点在地图上的栅格坐标
 * @param[in] search_parameters 搜索配置参数
 * @param[in] candidates 候选解
 * @param[in] candidate_depth 搜索树高度
 * @param[in] candidate_resolution 当前层的分辨率   
 * @param[in] min_score 候选点最小得分
 * @return Candidate2D 最优解
 */
Candidate2D FastCorrelativeScanMatcher2D::BranchAndBound(
      const std::vector<DiscreteScan2D>& discrete_scans,
      const SearchParameters& search_parameters,
      const std::vector<Candidate2D>& candidates, 
      const int candidate_depth,
      const int& candidate_resolution, 
      const int& angle_step, 
      float min_score) const {

  // 这个函数是以递归调用的方式求解的
  // 首先给出了递归终止的条件, 就是如果到了第0层(到底了), 意味着我们搜索到了一个叶子节点.
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  // 然后创建一个临时的候选解, 并将得分设置为min_score
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;

  // 搜索步长减为上层的一半
  const int half_width = candidate_resolution / 2;
  int half_theta_step = 1;
  if (angle_step > 1) {
    half_theta_step = angle_step / 2;  
  }
  
  // 遍历所有的候选点(枝)
  for (const Candidate2D& candidate : candidates) {
    //  Step: 剪枝   上界低于设置的阈值 或者 低于当前的最优解那么直接不用搜索这一个分支了
    // cartographer源代码这里剪枝的判断有误，递归过程中，下界得分会更新在best_high_resolution_candidate.score，
    //  而如果是 candidate.score <= min_score，那么这个下界不是当前的最新结果，因此会走到错误的分支上浪费时间！！
    // if (candidate.score <= min_score) {    // 有误 ！！！！
    if (candidate.score <= best_high_resolution_candidate.score) {
      break;
    }
    // 到这里说明 上界高与当前最优解，那么有可能这个枝存在得分高与最优的匹配得分的叶子，接下来需要对其进行分枝
    std::vector<Candidate2D> higher_resolution_candidates;

    // Step: 分枝 对x、y，theta偏移进行遍历, 求出candidate的八个子节点候选解
    for (int theta_offset : {-half_theta_step, 0, half_theta_step}) {
      if (candidate.scan_index + theta_offset > search_parameters.rotated_scans_num_ - 1) {
        break;
      }
      if (candidate.scan_index + theta_offset < 0) {
        continue;
      }

      for (int x_offset : {0, half_width}) { // 只能取0和half_width
        // // 如果超过了界限, 就跳过
        // if (candidate.x_index_offset + x_offset >
        //     search_parameters.linear_bounds_[candidate.scan_index + theta_offset].max_x) {
        //   break;
        // }

        for (int y_offset : {0, half_width}) {
          // if (candidate.y_index_offset + y_offset >
          //     search_parameters.linear_bounds_[candidate.scan_index].max_y) {
          //   break;
          // }

          // 候选者依次推进来, 一共4个,可以看出, 分枝定界方法的分枝是向右下角的四个子节点进行分枝
          higher_resolution_candidates.emplace_back(
              candidate.scan_index + theta_offset, candidate.x_index_offset + x_offset,
              candidate.y_index_offset + y_offset, search_parameters);
        }
      }
    }

    // 对新生成的枝求解上界并进行排序, 同一个点云, 不同地图
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);

    // 递归调用BranchAndBound对新生成的higher_resolution_candidates进行搜索 
    // 先对其分数最高的节点继续进行分支, 直到最底层, 然后再返回倒数第二层再进行迭代
    // 如果倒数第二层的最高分没有上一个的最底层（叶子层）的分数高, 则跳过, 
    // 否则继续向下进行分支与评分
 
    // Step: 定界 best_high_resolution_candidate.score
    // max 比较的是 score 
    // 返回得分更高的候选解 
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
          higher_resolution_candidates, candidate_depth - 1, candidate_resolution / 2,
          half_theta_step, best_high_resolution_candidate.score));
  }

  return best_high_resolution_candidate;
}
}  // namespace 
}  // namespace
