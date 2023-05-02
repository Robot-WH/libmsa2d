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
#include <Eigen/Geometry>
#include "GridMapBase.hpp"
#include "OccGridCell.hpp"
#include "../Sensor/LaserPointContainer.h"


namespace msa2d {
namespace map {

class OccGridMap : public GridMapBase<OccGridCell> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @param {float} map_resolution
     * @param {Vector2i} &size 
     * @param map_in_world map坐标系原点在world坐标系的坐标
     */    
    OccGridMap(float map_resolution, const Eigen::Vector2i &size, 
        const Eigen::Vector2f& map_in_world) 
        : GridMapBase<OccGridCell>(map_resolution, size, map_in_world) {
    }

    virtual ~OccGridMap() {}

    void updateSetOccupied(int index) {
        this->getCell(index).updateSetOccupied();
    }

    void updateSetFree(int index) {
        this->getCell(index).updateSetFree();
    }

    void updateUnsetFree(int index) {
        this->getCell(index).updateUnsetFree();
    }

    float getGridProbabilityMap(int index) const {
        return this->getCell(index).getGridProbability();
    }

    bool isOccupied(int xMap, int yMap) const {
        return this->getCell(xMap, yMap).isOccupied();
    }

    bool isFree(int xMap, int yMap) const {
        return (this->getCell(xMap, yMap).isFree());
    }

    bool isOccupied(int index) const {
        return (this->getCell(index).isOccupied());
    }

    bool isFree(int index) const {
        return (this->getCell(index).isFree());
    }

    // float getObstacleThreshold() const {
    //     _CellType temp;
    //     temp.resetGridCell();
    //     return grid_func_.getGridProbability(temp);
    // }

    // void setUpdateFreeFactor(float factor) {
    //     grid_func_.setUpdateFreeFactor(factor);
    // }

    // void setUpdateOccupiedFactor(float factor) {
    //     grid_func_.setUpdateOccupiedFactor(factor);
    // }
};
} // namespace Estimator2D
}
