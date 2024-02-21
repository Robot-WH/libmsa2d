#pragma once 
#include <iostream>
#include <unordered_map>
#include "robo2d/Sensor/point_cloud.hpp"
namespace robo2d {
namespace filter {

class OutLierFilter {
public:
    struct CellInfo {
        CellInfo() {}
        CellInfo(const uint16_t& point_index, const float& cost_v)
        : point_index_(point_index), cost_v_(cost_v) {}
        uint16_t point_index_;    // point在原始点云中的index
        float cost_v_;      // 距离cell中心的距离
    };
    VoxelGridFilter(float cell_size, float lidar_range);
    void Filter(sensor::LaserPointCloud& point_cloud);
private:
    uint16_t getCellIndex(const uint16_t& x, const uint16_t& y);
    float cell_size_;
    float lidar_range_;
    uint16_t map_length_, center_x_, center_y_;  
    std::unordered_map<uint16_t, CellInfo> hash_map_;  
};
}
}