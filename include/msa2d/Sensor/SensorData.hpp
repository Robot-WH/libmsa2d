
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
    Eigen::Vector3d gyro_;  
    Eigen::Quaterniond orientation_;  
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
}
}