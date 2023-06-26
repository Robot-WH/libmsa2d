
#pragma once 

#include <vector>
#include <memory>
#include "Eigen/Core"
// #include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"
#include "../common/Pose2d.hpp"

namespace msa2d {
namespace sensor {

/**
 * @brief 激光点结构
 * 
 */
struct LaserPoint {
    double rel_time_;  // 相对第一个点的时间戳  
    Eigen::Vector2f pos_;  // x, y 
    float range_; 
    uint16_t index_;  
};

/**
 * @brief 点云结构, 包含雷达一帧数据的所有数据点 
 * 
 */
template<typename _PointType>
class PointCloud {
 public:
  PointCloud() {}
  explicit PointCloud(std::vector<_PointType> points)
      : points_(std::move(points)) {}

  size_t size() const { return points_.size(); }
  bool empty() const { return points_.empty(); }

  // 返回vector的引用
  std::vector<_PointType>& points() {
      return points_;
  }

  const _PointType& operator[](const size_t index) const {
    return points_[index];
  }

  // Iterator over the points in the point cloud.
  using ConstIterator = typename std::vector<_PointType>::const_iterator;
  ConstIterator begin() const { return points_.begin(); }
  ConstIterator end() const { return points_.end(); }

  const _PointType& front() const {return points_.front();}
  const _PointType& back() const {return points_.back();}

  void push_back(_PointType value) {
      points_.push_back(std::move(value));
  }

  void reserve(const size_t& n) {
      points_.reserve(n); 
  }

  // Creates a PointCloud consisting of all the points for which `predicate`
  // returns true
  // 根据条件进行赋值
  template <class UnaryPredicate>
  PointCloud copy_if(UnaryPredicate predicate) const {
    std::vector<_PointType> points;

    // Note: benchmarks show that it is better to have this conditional outside
    // the loop.
    for (size_t index = 0; index < size(); ++index) {
      const _PointType& point = points_[index];
      // 表达式为true时才使用这个点
      if (predicate(point)) {
        points.push_back(point);
      }
    }

    return PointCloud(points);
  }
 private:
  // For 2D points, the third entry is 0.f.
  std::vector<_PointType> points_;
};

using LaserPointCloud = PointCloud<LaserPoint>;

struct LaserScan {
    using ptr = std::unique_ptr<LaserScan>; 
    using Ptr = std::shared_ptr<LaserScan>; 
    double start_time_;    
    double end_time_;    
    double scan_period_;   // 一帧的总时间
    // std::vector<LaserPoint> pointcloud_;
    LaserPointCloud pointcloud_;
};

/**
 * @brief 对输入的点云做坐标变换
 * 
 * @param[in] point_cloud 输入的点云
 * @param[in] transform 坐标变换
 * @return PointCloud 变换之后的点云
 */
static LaserPointCloud TransformPointCloud(const LaserPointCloud& point_cloud,
                               const Pose2d& transform) {
  std::vector<LaserPoint> points;
  points.reserve(point_cloud.size());
  for (const auto& point : point_cloud) {
    LaserPoint transform_point = point; 
    transform_point.pos_ = transform * point.pos_;
    points.emplace_back(transform_point);
  }
  return LaserPointCloud(points);
}

// // Transforms 'point_cloud' according to 'transform'.
// PointCloud TransformPointCloud(const PointCloud& point_cloud,
//                                const transform::Rigid3f& transform);

// // Transforms 'point_cloud' according to 'transform'.
// TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
//                                          const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
// PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
//                           float max_z);

}  // namespace 
}  // namespace 
