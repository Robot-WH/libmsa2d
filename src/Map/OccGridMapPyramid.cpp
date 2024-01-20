/**
 * @file OccGridMapPyramid.cpp
 * @author lwh (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <glog/logging.h>
#include "msa2d/Map/OccGridMapPyramid.h"
#include "msa2d/common/tic_toc.h"
#include "msa2d/Map/OccGridCnt.hpp"
#include "msa2d/Map/OccGridLogOdd.hpp"

namespace msa2d {
namespace map {

OccGridMapPyramid::OccGridMapPyramid(const Option& option) : option_(option) {
    std::cout << "创建占据栅格地图金字塔OccGridMapPyramid，底层分辨率：" 
        << option.bottom_resolution  << ", 底层地图的尺寸 X：" << option.map_sizeX  
        << ", Y: " <<  option.map_sizeY  << ", 金字塔层数：" << option.num_depth 
        << ",距离边界的最小距离：" << option.min_distance_to_boundary
        << ", 栅格的更新方法：" << option.grid_update_method  << std::endl;

    Eigen::Vector2i map_grid_size(option.map_sizeX / option.bottom_resolution , 
        option.map_sizeY / option.bottom_resolution ); // 第一层地图栅格size
    // 地图原点在世界坐标系下的坐标
    map_in_world_.x() = - option.map_sizeX  *  0.5;  
    map_in_world_.y() = - option.map_sizeY  * 0.5;  
    float mapResolution = option.bottom_resolution;

    for (unsigned int i = 0; i < option.num_depth; ++i) {
        std::cout << "map layer: " << i << ", cellLength: " << mapResolution
        << " length:" << map_grid_size.x() << " ,width: " << map_grid_size.y() << "\n";
        /** 创建占用栅格地图 **/
        OccGridMapBase* occ_grid_map = nullptr;

        if (option.grid_update_method == "cnt") {
            occ_grid_map = new OccGridMapImpl<OccGridCnt>(mapResolution, map_grid_size, 
                Eigen::Vector2f(map_in_world_.x(), map_in_world_.y()));
        } else if (option.grid_update_method == "probability") {
            occ_grid_map = new OccGridMapImpl<OccGridLogOdd>(mapResolution, map_grid_size, 
                Eigen::Vector2f(map_in_world_.x(), map_in_world_.y()));
        }
        
        OccGridMapContainer_.push_back(occ_grid_map);

        map_grid_size /= 2;       // 地图格子行、列格子数减半
        mapResolution *= 2.0f; // 地图精度减半
    }
}

OccGridMapPyramid::~OccGridMapPyramid() {
    unsigned int size = OccGridMapContainer_.size();

    for (unsigned int i = 0; i < size; ++i) {
        delete OccGridMapContainer_[i];
    } /// 析构函数，需释放使用的动态内存
}

void OccGridMapPyramid::reset() {
    unsigned int size = OccGridMapContainer_.size();

    for (unsigned int i = 0; i < size; ++i) {
        OccGridMapContainer_[i]->reset(); // 重置地图
    }
}

float OccGridMapPyramid::getScaleToMap() const { 
    return OccGridMapContainer_[0]->getGridMapBase().getScaleToMap(); 
} // 获取地图尺度 scale = 1.0 / map_resolution.

int OccGridMapPyramid::getMapLevels() const { 
    return OccGridMapContainer_.size(); 
}

OccGridMapBase* OccGridMapPyramid::getGridMap(int mapLevel) { 
    return OccGridMapContainer_[mapLevel]; 
}

std::mutex* OccGridMapPyramid::getMapMutex(int i) {
    return OccGridMapContainer_[i]->getMapMutex(); /// 获取指定层的地图锁
}

void OccGridMapPyramid::onMapUpdated() {
    /// 提示地图已经得到更新，cache中的临时数据无效
    unsigned int size = OccGridMapContainer_.size();

    for (unsigned int i = 0; i < size; ++i) {
        // OccGridMapContainer_[i].resetCachedData();
    }
}

void OccGridMapPyramid::updateByScan(const std::vector<sensor::LaserPointContainer>& data_containers, 
                                                            const Eigen::Vector3f &laser_pose_in_world) {
    // 判断地图是否需要移动
    if (isCloseToBoundary(laser_pose_in_world)) {
        std::cout << color::GREEN << "进入submap边界，submap进行移动" 
            << color::RESET << std::endl;
        // 计算map移动后原点的世界坐标
        // map的中点移动到当前laser处
        Eigen::Vector2f new_map_pos_in_world{laser_pose_in_world[0] - option_.map_sizeX  * 0.5f,
                                                                                                laser_pose_in_world[1] - option_.map_sizeY  * 0.5f}; 
        // 移动地图金字塔
        // time::TicToc tt;
        moveTo(new_map_pos_in_world); 
        // tt.toc("moveTo");
        map_in_world_ = new_map_pos_in_world;
    } 
    std::cout << "isCloseToBoundary() done" << std::endl;
    unsigned int size = data_containers.size();
    for (unsigned int i = 0; i < size; ++i) {
        OccGridMapContainer_[i]->updateByScan(data_containers[i].dataPoints, laser_pose_in_world);
    }
    //std::cout << "\n";
}

bool OccGridMapPyramid::isCloseToBoundary(const Eigen::Vector3f& laser_pose_in_world) {
    Eigen::Vector2f laser_ref_map = {laser_pose_in_world[0] - map_in_world_[0], 
                                                                        laser_pose_in_world[1] - map_in_world_[1]};  
    if (laser_ref_map[0] < option_.min_distance_to_boundary || 
            laser_ref_map[0] > option_.map_sizeX - option_.min_distance_to_boundary ||
            laser_ref_map[1] < option_.min_distance_to_boundary || 
            laser_ref_map[1] > option_.map_sizeY - option_.min_distance_to_boundary) {
            return true;  
    }
    return false;   
}

void OccGridMapPyramid::moveTo(const Eigen::Vector2f& new_map_pos_in_world) {
    for (uint16_t i = 0; i < OccGridMapContainer_.size(); ++i) {
        std::cout << "OccGridMapPyramid::moveTo, i: " << i << std::endl;
        OccGridMapContainer_[i]->moveTo(new_map_pos_in_world);
        std::cout << "OccGridMapPyramid::moveTo, done" << i << std::endl;
    }
}

void OccGridMapPyramid::setUpdateFactorFree(float free_factor) {
    size_t size = OccGridMapContainer_.size();

    for (unsigned int i = 0; i < size; ++i) {
        // OccGridMap &map = OccGridMapContainer_[i]->getGridMap();
        // map.setUpdateFreeFactor(free_factor);
    }
}

void OccGridMapPyramid::setUpdateFactorOccupied(float occupied_factor) {
    size_t size = OccGridMapContainer_.size();

    for (unsigned int i = 0; i < size; ++i) {
        // OccGridMap& map = OccGridMapContainer_[i]->getGridMap();
        // map.setUpdateOccupiedFactor(occupied_factor);
    }
}
} // namespace 
}

