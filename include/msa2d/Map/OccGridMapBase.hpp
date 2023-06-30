/**
 * @file OccGridMapBase.hpp
 * @author lwh(you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-10-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include <iostream>
#include <Eigen/Geometry>
#include <mutex>
#include "GridMapImpl.hpp"
#include "../Sensor/LaserPointContainer.h"
#include "../common/UtilFunctions.hpp"
namespace msa2d {
namespace map {

class OccGridMapBase {
public:
    OccGridMapBase() : mapModifyMutex_(new std::mutex()) {}
    /***********************************下面是针对占据栅格所新增的函数*************************************/
    virtual ~OccGridMapBase() {
        delete mapModifyMutex_;  
    }

    virtual void updateSetOccupied(int index) = 0;
    virtual void updateSetFree(int index) = 0;
    virtual void updateUnsetFree(int index) = 0; 
    virtual const float& getGridProbability(const int& index) = 0;
    virtual const float& getGridProbability(const int& xMap, const int& yMap) = 0;
    virtual bool isOccupied(int xMap, int yMap) const = 0;
    virtual bool isFree(int xMap, int yMap) const = 0;
    virtual bool isOccupied(int index) const = 0; 
    virtual bool isFree(int index) const  = 0;
    virtual bool isUnknow(int index) const  = 0;
    virtual bool isUnknow(int xMap, int yMap) const  = 0;
    virtual void reset() = 0;
    virtual void clear() = 0; 
    virtual void updateByScan(const std::vector<Eigen::Vector2f>& laser_same_scale, 
            const Eigen::Vector3f& scan_pose_in_world) = 0;  
    virtual void moveTo(const Eigen::Vector2f& new_map_pos_in_world) = 0;
    virtual const GridMapBase& getGridMapBase() const = 0;  

    /// 获取当前地图的互斥锁
    std::mutex* getMapMutex() {
        return mapModifyMutex_;
    }
protected:
    std::mutex* mapModifyMutex_ = nullptr;              // 地图锁，修改地图时，需要加锁避免多线程资源访问冲突。
};
} // namespace Estimator2D
}
