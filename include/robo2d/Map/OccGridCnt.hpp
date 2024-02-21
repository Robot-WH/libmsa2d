/**
 * @file OccGridCnt.hpp
 * @author lwh (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-10-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include <glog/logging.h>
#include <cmath>
namespace robo2d {
namespace map {
class OccGridCnt {
public:
    /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
    OccGridCnt() {
        free_t_ = 1;
        occupied_t_ = 1; 
        free_factor_ = 5;
        occupied_factor_ = 5; 
    }

    /**
   * Update cell as occupied
   * @param cell The cell.
   */
    void updateSetOccupied() {
        if (occupied_t_ < 1000) {
            occupied_t_ += occupied_factor_;
        }
        if (free_t_ > free_factor_) {
            free_t_ -= free_factor_;  
        }
    }

    /**
   * Update cell as free
   * @param cell The cell.
   */
    void updateSetFree() {
        if (free_t_ < 1000) {
            free_t_ += free_factor_;
        }
        if (occupied_t_ > occupied_factor_) {
            occupied_t_ -= occupied_factor_;  
        }
    }

    /**
     * @brief 由于占用的优先级高与free，而更新栅格时，可能出现先观测到free，后
     *                  观测到占用的情况，此时就需要将之前的free取消
     * 
     */
    void updateUnsetFree() {
        if (occupied_t_ < 1000) {
            occupied_t_ += occupied_factor_;
        }
        if (free_t_ > free_factor_) {
            free_t_ -= free_factor_;  
        }
    }

    /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
    float getGridProbability() const {
        return static_cast<float>(occupied_t_) / (occupied_t_ + free_t_);
    }

    /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
    bool isOccupied() const {
        return occupied_t_ > free_t_;
    }

    bool isFree() const {
        return occupied_t_ < free_t_;
    }

        /**
     * @brief 返回栅格是否未知
     * 
     * @return true 
     * @return false 
     */
    bool isUnknow() const {
        return false;
    }

    /**
   * Reset Cell to prior probability.
   */
    void resetGridCell() {
        free_t_ = 1;
        occupied_t_ = 1; 
    }

    void setUpdateFreeFactor(float factor) {
        free_factor_ = factor;
    }

    void setUpdateOccupiedFactor(float factor) {
        occupied_factor_ = factor;  
    }

protected:
    bool update_ = false;  
    uint16_t free_t_;
    uint16_t occupied_t_;  
    uint16_t free_factor_;
    uint16_t occupied_factor_;  
};
}
}