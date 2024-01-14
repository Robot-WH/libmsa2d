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

/**
 * @brief 对激光点进行体素降采样滤波    同时要保证点顺序是从小到大  
 * 
 * @param point_cloud 
 */
// void VoxelGridFilter::Filter(sensor::LaserPointCloud& point_cloud) {
//     hash_map_.clear();
//     std::vector<uint16_t> cell_seq;
//     cell_seq.reserve(point_cloud.size()); 

//     for (uint16_t i = 0; i < point_cloud.size(); ++i) {
//         float map_x = (point_cloud[i].pos_.x() + lidar_range_) / cell_size_; 
//         float map_y = (point_cloud[i].pos_.y() + lidar_range_) / cell_size_; 
//         float cost_v = std::pow(map_x - (uint16_t)map_x, 2) + 
//                                     std::pow(map_y - (uint16_t)map_y, 2);
//         uint16_t key = getCellIndex((uint16_t)map_x, (uint16_t)map_y);

//         if (hash_map_.count(key)) {
//             if (hash_map_[key].cost_v_ > cost_v) {
//                 hash_map_[key].cost_v_ = cost_v;
//                 hash_map_[key].point_index_ = i;  
//             }
//         } else {
//             hash_map_[key] = CellInfo(i, cost_v);
//             cell_seq.push_back(key);
//         }
//     }
//     sensor::LaserPointCloud filtered_pointcloud;
//     filtered_pointcloud.reserve(hash_map_.size());
//     // 提取点云
//     for (const uint16_t& i : cell_seq) {
//         filtered_pointcloud.push_back(point_cloud[hash_map_[i].point_index_]); 
//     }

//     point_cloud = std::move(filtered_pointcloud);
// }

void VoxelGridFilter::Filter(sensor::LaserPointCloud& point_cloud) {
    sensor::LaserPointCloud filtered_pointcloud;
    filtered_pointcloud.reserve(point_cloud.size());
    hash_map_.clear();
    int last_key = -1; 
    float best_score = -1.0f;
    uint16_t best_index = 0;  

    for (uint16_t i = 0; i < point_cloud.size(); ++i) {
        float map_x = (point_cloud[i].pos_.x() + lidar_range_) / cell_size_; 
        float map_y = (point_cloud[i].pos_.y() + lidar_range_) / cell_size_; 
        float cost_v = std::pow(map_x - (uint16_t)map_x, 2) + 
                                    std::pow(map_y - (uint16_t)map_y, 2);
        uint16_t key = getCellIndex((uint16_t)map_x, (uint16_t)map_y);
        // 表示当前进入了新的cell中，那么之前的cell需要提取一个点
        if (key != last_key) {
            if (best_score > 0) {
                filtered_pointcloud.push_back(point_cloud[best_index]); 
            }
            best_score = cost_v;
            best_index = i;  
            last_key = key;  
        } else {
            if (cost_v < best_score) {
                best_score = cost_v;
                best_index = i;  
            }
        }

        // if (hash_map_.count(key)) {
        //     if (hash_map_[key].cost_v_ > cost_v) {
        //         hash_map_[key].cost_v_ = cost_v;
        //         hash_map_[key].point_index_ = i;  
        //     }
        // } else {
        //     hash_map_[key] = CellInfo(i, cost_v);
        // }
    }


    // sensor::LaserPointCloud filtered_pointcloud;
    // filtered_pointcloud.reserve(hash_map_.size());
    // // 提取点云
    // for (const auto& pair : hash_map_) {
    //     uint16_t index = pair.second.point_index_;
    //     filtered_pointcloud.push_back(point_cloud[index]); 
    // }
    point_cloud = std::move(filtered_pointcloud);
}

uint16_t VoxelGridFilter::getCellIndex(const uint16_t& x, const uint16_t& y) {
    return y * map_length_ + x;  
}

}
}