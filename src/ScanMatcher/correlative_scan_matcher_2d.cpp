#include <cmath>
#include <iostream>
#include "msa2d/ScanMatcher/correlative_scan_matcher_2d.h"

namespace msa2d {
namespace ScanMatcher {

/**
 * @brief Construct a new Search Parameters:: Search Parameters object
 * 
 * @param linear_search_window 单边平移搜索空间大小  单位m
 * @param angular_search_window  单边角度搜索空间大小  
 * @param point_cloud 
 * @param resolution 栅格地图的分辨率
 * @param expansion_coeff 实际匹配地图金字塔最低层的膨胀系数 (实际分辨率 = resolution * expansion_coeff)
 */
SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::LaserPointCloud& point_cloud,
                                   const double resolution, const int& expansion_coeff,
                                   const int& map_depth) : resolution_(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;

  // 求得 point_cloud 中雷达数据的 最大的值（最远点的距离）
  for (const auto& point : point_cloud) {
    const float range = point.pos_.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }

  // 根据论文里的公式 求得角度分辨率 angular_perturbation_step_size
  /**
   * @details 这个角度增量通过余弦定理计算
   */
  const double kSafetyMargin = 1. - 1e-3;   // 缩小一点的系数  
  angular_perturbation_step_ =
      kSafetyMargin * std::acos(1. - math::Pow2( resolution_ * expansion_coeff) /
                                         (2. * math::Pow2(max_scan_range)));
  /**
   * @brief 计算每一层的角度分辨率，但是其实角度很小，因此角度分辨率的变化近似于栅格分辨率的变化 
   */
  // angular_perturbation_steps_.push_back(1);
  // std::cout << "angular_perturbation_step_: " << angular_perturbation_step_ << std::endl;

  // for (int i = 1; i < map_depth; ++i) {
  //   std::cout << "curr res: " << resolution_ * expansion_coeff * (1 << i) << std::endl;
  //   float angular_perturbation_step = 
  //     kSafetyMargin * std::acos(1. - math::Pow2(resolution_ * expansion_coeff * (1 << i)) /
  //                                        (2. * math::Pow2(max_scan_range)));
  //   std::cout << "angular_perturbation_step_: " << angular_perturbation_step << std::endl;
  //   angular_perturbation_steps_.push_back(angular_perturbation_step / angular_perturbation_step_); 
  // }

  // for (auto& v : angular_perturbation_steps_) {
  //   std::cout << "v: " << v << std::endl;
  // }

  // 范围除以分辨率得到个数     ceil向上取整数   
  angular_search_step_num_one_side_ =
      std::ceil(angular_search_window / angular_perturbation_step_);
  // num_scans是要生成旋转点云的个数, 将 num_angular_perturbations 扩大了2倍
  // +-两个方向 都要搜索 
  rotated_scans_num_ = 2 * angular_search_step_num_one_side_ + 1;

  // XY方向的搜索范围, 单位是多少个栅格
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution_);

  // linear_bounds 的作用是确定不同旋转点云的平移搜索的最大最小边界
  linear_bounds_.reserve(rotated_scans_num_);
  for (int i = 0; i != rotated_scans_num_; ++i) {
    linear_bounds_.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}


/**
 * @brief 收缩搜索范围    保证每一个激光点都要在地图范围内    
 *            计算每一帧点云 在保证最后一个点能在地图范围内时 的最大移动范围
 * @param scans 
 * @param map 
 */
bool SearchParameters::ShrinkToFit(const LinearBounds& map_bounds,
                                   const map::GridMapBase* map, const Pose2d& initial_pose_estimate) {
  // // CHECK_EQ(scans.size(), rotated_scans_num_);
  // // CHECK_EQ(linear_bounds_.size(), rotated_scans_num_);

  // // 遍历生成的旋转后的很多scan
  // // for (int i = 0; i != rotated_scans_num_; ++i) {
  // for (int i = 0; i < 1; ++i) {
  //   Eigen::Array2i min_bound(-map->getGridSizeX(), -map->getGridSizeY());
  //   Eigen::Array2i max_bound(map->getGridSizeX(), map->getGridSizeY());

  //   // 对每个不同旋转的点云的每一个点进行遍历, 确定这帧点云的最大最小的坐标索引
  //   for (const Eigen::Array2i& xy_index : scans[i]) {
  //     // Array2i.min的作用是 获取对应元素的最小值组成新的Array2i
  //     // 为什么我感觉这里min,max用反了？？？？？？？？？？？？？？？？？？？？
  //     /**
  //      * @todo 这里错了  
  //      * 
  //      */
  //     // min_bound = min_bound.min(-xy_index);
  //     // max_bound = max_bound.max(Eigen::Array2i(map->getGridSizeX() - 1,
  //     //                                          map->getGridSizeY() - 1) - xy_index);
  //     min_bound = min_bound.max(-xy_index);
  //     max_bound = max_bound.min(Eigen::Array2i(map->getGridSizeX() - 1,
  //                                              map->getGridSizeY() - 1) - xy_index);
  //     // std::cout << "xy_index: " << xy_index.transpose() << std::endl;

  //     // std::cout << "min_bound.x(): " << min_bound.x() << std::endl;
  //     // std::cout << "min_bound.y(): " << min_bound.y() << std::endl;
  //     // std::cout << "max_bound.x(): " << max_bound.x() << std::endl;
  //     // std::cout << "max_bound.y(): " << max_bound.y() << std::endl;
  //   }
  //   std::cout << "linear_bounds_[i] i: " << i << ",min_x: " << linear_bounds_[i].min_x 
  //   << "min_bound.x(): " << min_bound.x() << ", min_y: " << linear_bounds_[i].min_y
  //   << "min_bound.y(): " << min_bound.y() << ", max_x: " << linear_bounds_[i].max_x 
  //   << ",max_bound.x(): " << max_bound.x() << ", max_y: " << linear_bounds_[i].max_y 
  //    << ",max_bound.y(): " << max_bound.y() << std::endl;
  //   // 计算每一帧点云 在保证最后一个点能在地图范围内时 的最大移动范围
  //   linear_bounds_[i].min_x = std::max(linear_bounds_[i].min_x, min_bound.x());
  //   linear_bounds_[i].max_x = std::min(linear_bounds_[i].max_x, max_bound.x());
  //   linear_bounds_[i].min_y = std::max(linear_bounds_[i].min_y, min_bound.y());
  //   linear_bounds_[i].max_y = std::min(linear_bounds_[i].max_y, max_bound.y());

  // }
  LinearBounds map_bound{0, map->getGridSizeX(), 0, map->getGridSizeY()};
  // 求解栅格地图的范围
  // 如果栅格地图的范围太小，那么没必要进行全局匹配
  if (map_bound.max_x - map_bound.min_x < 1 * map->getScaleToMap()
        && map_bound.max_y - map_bound.min_y < 1 * map->getScaleToMap()) {
    return false;  
  }
  // Eigen::Array2i init_pos_grid_index = map->GetGridIndex(
  //   Eigen::Vector2f(initial_pose_estimate.x(), initial_pose_estimate.y())); 
  Eigen::Array2i init_pos_grid_index(initial_pose_estimate.x(), initial_pose_estimate.y()); 

  linear_bounds_[0].min_x = std::max(linear_bounds_[0].min_x, 
                                                                              map_bound.min_x - init_pos_grid_index.x());
  linear_bounds_[0].max_x = std::min(linear_bounds_[0].max_x, 
                                                                               map_bound.max_x - init_pos_grid_index.x());
  linear_bounds_[0].min_y = std::max(linear_bounds_[0].min_y, 
                                                                              map_bound.min_y - init_pos_grid_index.y());
  linear_bounds_[0].max_y = std::min(linear_bounds_[0].max_y, 
                                                                               map_bound.max_y - init_pos_grid_index.y());

      // std::cout << "linear_bounds_[0].min_x: " << linear_bounds_[0].min_x << std::endl;
      // std::cout << "linear_bounds_[0].max_x: " << linear_bounds_[0].max_x << std::endl;
      // std::cout << "linear_bounds_[0].min_y: " << linear_bounds_[0].min_y << std::endl;
      // std::cout << "linear_bounds_[0].max_y: " << linear_bounds_[0].max_y << std::endl;

  for (int i = 1; i != rotated_scans_num_; ++i) {
    linear_bounds_[i] = linear_bounds_[0];
  }
  return true; 
}


// 生成按照不同角度旋转后的点云集合
std::vector<sensor::LaserPointCloud> GenerateRotatedScans(
    const sensor::LaserPointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<sensor::LaserPointCloud> rotated_scans;
  // 生成 num_scans 个旋转后的点云
  rotated_scans.reserve(search_parameters.rotated_scans_num_);
  // 起始角度
  double delta_theta = -search_parameters.angular_search_step_num_one_side_ *
                       search_parameters.angular_perturbation_step_;
  // 进行遍历，生成旋转不同角度后的点云集合
  for (int scan_index = 0; scan_index < search_parameters.rotated_scans_num_;
       ++scan_index, delta_theta += search_parameters.angular_perturbation_step_) {
    // 将 point_cloud 绕Z轴旋转了delta_theta
          /**
       * @todo  注意这里 !!!!!!!!!!!!!!!!!!!!!!
       * 
       */
    rotated_scans.push_back(sensor::TransformPointCloud(
        point_cloud, Pose2d(0, 0, delta_theta)));
  }
  return rotated_scans;
}

/**
 * @brief 将旋转后的点云集合施加一个initial_translation的平移, 获取平移后的点在地图中的索引
 * 
 * @param map 
 * @param scans 各个旋转角度的点云 
 * @param initial_translation 
 * @return std::vector<DiscreteScan2D> 
 */
std::vector<DiscreteScan2D> DiscretizeScans(
    const map::GridMapBase* map, const std::vector<sensor::LaserPointCloud>& scans,
    const Pose2d& initial_pose_estimate) {
  // discrete_scans的size 为 旋转的点云的个数
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());

  for (const sensor::LaserPointCloud& scan : scans) {
    // discrete_scans中的每一个 DiscreteScan2D 的size设置为这一帧点云中所有点的个数
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());

    // 点云中的每一个点进行平移, 获取平移后的栅格索引
    for (const auto& point : scan) {
      // 对scan中的每个点进行平移
      // std::cout << "point.pos_: " << point.pos_.transpose() << ", range: " << point.range_ << std::endl;
      // const Eigen::Vector2f translated_point =
      //     Eigen::Affine2f(initial_translation) * point.pos_;

      // discrete_scans.back().push_back(map->GetGridIndex(translated_point));
      Eigen::Array2i point_in_map = map->GetGridIndex(point.pos_);
      const Eigen::Vector2i translated_point(point_in_map.x() + initial_pose_estimate.x(),
                                                                                        point_in_map.y() + initial_pose_estimate.y());

      discrete_scans.back().push_back(std::move(translated_point));
      // std::cout << "GridIndex: " << map->GetGridIndex(translated_point).transpose() << std::endl;
    }
  }
  return discrete_scans;
}

}  // namespace 
}  // namespace 
