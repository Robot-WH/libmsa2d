
#pragma once 
#include <vector>
#include <Eigen/Core>
#include <memory>
#include "../common/Pose2d.hpp"

namespace msa2d {
namespace sensor {

struct ImuData {
    double time_stamp_ = 0;
    Eigen::Vector3d acc_;
    Eigen::Vector3d angular_v_;  
};

struct WheelOdom {
    using ptr = std::unique_ptr<WheelOdom>;
    double time_stamp_ = 0;
    // 运动速度
    double v_x_ = 0, v_y_ = 0;  // 线速度
    double omega_yaw_ = 0;  // 角速度  
    // pose
    Pose2d pose_; 
};

struct LaserPoint {
    double rel_time_;  // 相对第一个点的时间戳  
    Eigen::Vector2f pos_;  // x, y 
    float range_; 
    uint16_t index_;  
};

struct LaserPointCloud {
    using ptr = std::unique_ptr<LaserPointCloud>; 
    using Ptr = std::shared_ptr<LaserPointCloud>; 
    double start_time_;    
    double end_time_;    
    double scan_period_;   // 一帧的总时间
    std::vector<LaserPoint> pointcloud_;
};

}
}