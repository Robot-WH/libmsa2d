/**
 * @file voxelgrid_filter.cpp
 * @author lwh ()
 * @brief 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "msa2d/Filter/voxelgrid_filter.h"

namespace msa2d {
namespace filter {

VoxelGridFilter::VoxelGridFilter(float cell_size, float lidar_range) 
: cell_size_(cell_size), lidar_range_(lidar_range) {
    map_length_ = std::ceil(2 * lidar_range / cell_size); 
}

void VoxelGridFilter::Filter(sensor::LaserPointCloud& point_cloud) {
    hash_map_.clear();
    for (uint16_t i = 0; i < point_cloud.size(); ++i) {
        float map_x = (point_cloud[i].pos_.x() + lidar_range_) / cell_size_; 
        float map_y = (point_cloud[i].pos_.y() + lidar_range_) / cell_size_; 
        float cost_v = std::pow(map_x - (uint16_t)map_x, 2) + 
                                    std::pow(map_y - (uint16_t)map_y, 2);
        uint16_t key = getCellIndex((uint16_t)map_x, (uint16_t)map_y);

        if (hash_map_.count(key)) {
            if (hash_map_[key].cost_v_ > cost_v) {
                hash_map_[key].cost_v_ = cost_v;
                hash_map_[key].point_index_ = i;  
            }
        } else {
            hash_map_[key] = CellInfo(i, cost_v);
        }
    }
    sensor::LaserPointCloud filtered_pointcloud;
    filtered_pointcloud.reserve(hash_map_.size());
    // 提取点云
    for (const auto& pair : hash_map_) {
        uint16_t index = pair.second.point_index_;
        filtered_pointcloud.push_back(point_cloud[index]); 
    }
    point_cloud = std::move(filtered_pointcloud);
}

uint16_t VoxelGridFilter::getCellIndex(const uint16_t& x, const uint16_t& y) {
    return y * map_length_ + x;  
}

}
}