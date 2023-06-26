// #include "msa2d/ScanMatcher/fast_correlative_scan_matcher_2d.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "glog/logging.h"


class SlidingWindowMaximum {
 public:
    // 添加值, 会将小于填入值的其他值删掉, 再将这个值放到最后
    void AddValue(const float value) {
        while (!non_ascending_maxima_.empty() &&
              value > non_ascending_maxima_.back()) {
          non_ascending_maxima_.pop_back();
        }
        non_ascending_maxima_.push_back(value);
    }

  // 删除值, 如果第一个值等于要删除的这个值, 则将这个值删掉
  void RemoveValue(const float value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  // 获取最大值, 因为是按照顺序存储的, 第一个值是最大的
  float GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.
  std::deque<float> non_ascending_maxima_;
};

class PrecomputationGrid2D {
 public:
  PrecomputationGrid2D(Eigen::MatrixXi map, const int cell_width,
      std::vector<float>* reusable_intermediate_grid);

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

// Probabilites mapped to 0 to 255.
  std::vector<int> cells_;   // 不同分辨率的栅格地图
    // Size of the precomputation grid.
  const Eigen::Vector2i map_grid_size_;    // 地图x方向与y方向的格子数

 private:
  uint8_t ComputeCellValue(float probability) const;

  // Offset of the precomputation grid in relation to the 'grid'
  // including the additional 'width' - 1 cells.
  const Eigen::Array2i offset_;

  const float min_score_;
  const float max_score_;
};

// 构造不同分辨率的地图
PrecomputationGrid2D::PrecomputationGrid2D(
    Eigen::MatrixXi map, const int cell_width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-cell_width + 1, -cell_width + 1),
      map_grid_size_(map.cols() + cell_width - 1,
                   map.rows() + cell_width - 1),
      min_score_(0.1f), // 0.1 min_score_
      max_score_(0.9f) { // 0.9 max_score_

    cells_.resize(map_grid_size_.x() * map_grid_size_.y());
    std::cout << "map_grid_size_ x: " << map_grid_size_.x() << ",y: " << map_grid_size_.y() << std::endl;
  std::cout << "cell_ size: " << cells_.size() << std::endl;
  
  CHECK_GE(cell_width, 1);    // >=1 
  Eigen::Vector2i origin_map_grid_size(map.cols(), map.rows()); 
  const int stride = map_grid_size_.x();
  std::cout << "origin_map_grid_size: " << origin_map_grid_size.transpose() << std::endl;
  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + cell_width.
  std::vector<float>& intermediate = *reusable_intermediate_grid;
  intermediate.resize(map_grid_size_.x() * origin_map_grid_size.y());   // 临时的地图数据
  
  // 先对x方向进行分辨率膨胀
  // 对每一行从左到右横着做一次滑窗, 将滑窗后的地图放在intermediate(临时数据)中
  std::cout << "build intermediate: " << std::endl;
  for (int y = 0; y != origin_map_grid_size.y(); ++y) {
    SlidingWindowMaximum current_values;
    // 获取 grid 的x坐标的索引: 首先获取 (0, y)
    current_values.AddValue(map(y, 0));  // 添加占据的概率 
    std::cout << "y: " << y << ", add map(y, 0): " << map(y, 0) << std::endl;
    // Step: 1 滑动窗口在x方向开始划入地图, 所以只进行 填入值
    // intermediate的索引x + cell_width - 1 + y * stride的范围是 [0, cell_width-2] 再加上 y * stride
    // grid的索引 x + cell_width 的坐标范围是 [1, cell_width-1]
    for (int x = -cell_width + 1; x != 0; ++x) {
      intermediate[x + cell_width - 1 + y * stride] = current_values.GetMaximum();
      if (x + cell_width < origin_map_grid_size.x()) {
        current_values.AddValue(map(y, x + cell_width));
        std::cout << "add map(y, x + cell_width): " << map(y, x + cell_width) << std::endl;
      }
    }

    // Step: 2 滑动窗口已经完全在地图里了, 滑窗进行一入一出的操作
    // x + cell_width - 1 + y * stride 的范围是 [cell_width-1, limits.num_x_cells-2] 再加上 y * stride
    // grid的索引 x + cell_width 的坐标范围是 [cell_width, limits.num_x_cells-cell_width-1]
    for (int x = 0; x < origin_map_grid_size.x() - cell_width; ++x) {
      intermediate[x + cell_width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(map(y, x));
      current_values.AddValue(map(y, x + cell_width));
      std::cout << "add map(y, x + cell_width): " << map(y, x + cell_width) << std::endl;
    }

    // Step: 3 滑动窗口正在划出, 一次减少一个值, 所以intermediate的宽度比grid多 cell_width-1
    // x + cell_width - 1 + y * stride 的范围是 [limits.num_x_cells-1, limits.num_x_cells+cell_width-1] 再加上 y * stride
    // grid 的索引 x的范围是 [limits.num_x_cells-cell_width, limits.num_x_cells-1]
    for (int x = std::max(origin_map_grid_size.x() - cell_width, 0);
         x != origin_map_grid_size.x(); ++x) {
      intermediate[x + cell_width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(map(y, x));
    }
    // 理论上, 滑窗走完地图的一行之后应该是空的, 经过 只入, 一出一入, 只出, 3个步骤
    current_values.CheckIsEmpty();
  }
  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
    std::cout << "intermediate: " << std::endl;
    for (int y = 0; y < origin_map_grid_size.y(); ++y) {
        for (int x = 0; x < map_grid_size_.x(); ++x) {
            std::cout << intermediate[x + y * stride] << ",";
        }
        std::cout << std::endl;
    }

//   // 根据intermediate的值, 对每一列从下到上竖着再做一次滑窗, 这个才是真正的地图cells_
  for (int x = 0; x != map_grid_size_.x(); ++x) {
    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);

    std::cout << "x: " << x << ", add intermediate[x]: " << intermediate[x] << std::endl;

    for (int y = -cell_width + 1; y != 0; ++y) {
      cells_[x + (y + cell_width - 1) * stride] = current_values.GetMaximum();
      std::cout << "add intermediate[x + (y + cell_width) * stride]: " << intermediate[x + (y + cell_width) * stride] << std::endl;
      if (y + cell_width < origin_map_grid_size.y()) {
        current_values.AddValue(intermediate[x + (y + cell_width) * stride]);
      }
    }

    for (int y = 0; y < origin_map_grid_size.y() - cell_width; ++y) {
      cells_[x + (y + cell_width - 1) * stride] = current_values.GetMaximum();
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + cell_width) * stride]);
    }

    for (int y = std::max(origin_map_grid_size.y() - cell_width, 0);
         y != origin_map_grid_size.y(); ++y) {
      cells_[x + (y + cell_width - 1) * stride] = current_values.GetMaximum();
      current_values.RemoveValue(intermediate[x + y * stride]);
    }

    current_values.CheckIsEmpty();
  }
}


int main() {
    Eigen::Matrix<int, 5, 4> m;
    m  << 3, 1, 0, 2,      // 0行
                0, 0, 0, 0,
                0, 4, 0, 0,
                0, 0, 5, 0,
                0, 0, 0, 0;

    for (int i = 0; i < m.rows(); i++)
        std::cout << "m row: " << m.row(i) << std::endl;
    
    std::cout << "m : " << std::endl << m.matrix() << std::endl;
    std::cout << "m(0, 0) : " << std::endl << m(0, 0) << std::endl;

    std::vector<float>* intermediate_grid = new std::vector<float>(); 
    PrecomputationGrid2D grid(m, 3, intermediate_grid);

    for (int i = 0; i < grid.map_grid_size_.y(); ++i) {
        for (int j = 0; j < grid.map_grid_size_.x(); ++j) {
            std::cout << grid.cells_[j + i * grid.map_grid_size_.x()] << ","; 
        }
        std::cout << std::endl;
    }

    return 0;
}