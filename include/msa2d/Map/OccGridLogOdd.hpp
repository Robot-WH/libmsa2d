/**
 * @file OccGridLogOdd.hpp
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

namespace msa2d {
namespace map {

/**
 * @brief 基于对数概率法更新的占据栅格  
 * 
 */
class OccGridLogOdd {
public:
    /**
     * @brief Construct a new Occ Grid Log Odd object
     * 
     */
    OccGridLogOdd() {
        logOddsVal = 0.0f;  
        prob_update_ = true;   // 概率未知  需要更新
        occ_prob_ = 0.5f;  
        // 设置free/占据 的栅格的对数概率 
        setUpdateFreeFactor(0.4f);   // 空闲的更新量     -0.42
        setUpdateOccupiedFactor(0.9f);   // 占据的更新量  0.41
    }

    /**
     * @brief 该栅格设置一次占据  
     * 
     */
    void updateSetOccupied() {
        if (logOddsVal < 50.0f) {
            logOddsVal += logOddsOccupied;
            prob_update_ = true;       // 该栅格概率需要更新 
        }
    }

    /**
     * @brief 该栅格设置一次空闲
     * 
     */
    void updateSetFree() {
        logOddsVal += logOddsFree;
        prob_update_ = true;       // 该栅格概率需要更新 
    }

    /**
     * @brief 该栅格取消一次空闲   
     * 
     */
    void updateUnsetFree() {
        logOddsVal -= logOddsFree;
        prob_update_ = true;       // 该栅格概率需要更新 
    }

    /**
     * @brief Get the Grid Probability object
     * 
     * @return float 
     */
    const float& getGridProbability() {
        // 如果概率更新了那么需要重新计算概率 ，否则直接将缓存的概率值输出 
        if (prob_update_) {
            float odds = exp(logOddsVal);
            occ_prob_ = odds / (odds + 1.0f);
            prob_update_ = false;   
        }
        return occ_prob_;
    }

    /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
    bool isOccupied() const {
        return logOddsVal > 0.0f;   // 即概率大于0.5 认为占据
    }

    /**
     * @brief 返回栅格是否空闲  
     * 
     * @return true 
     * @return false 
     */
    bool isFree() const {
        return logOddsVal < 0.0f;
    }

    /**
     * @brief 返回栅格是否未知
     * 
     * @return true 
     * @return false 
     */
    bool isUnknow() const {
        return logOddsVal == 0.0f;
    }

    /**
   * Reset Cell to prior probability.
   */
    void resetGridCell() {
        logOddsVal = 0.0f;
        prob_update_ = true;       // 该栅格概率已更新 
    }

    void setUpdateFreeFactor(float factor) {
        logOddsFree = probToLogOdds(factor);
    }

    void setUpdateOccupiedFactor(float factor) {
        logOddsOccupied = probToLogOdds(factor);    //  factor: 0.9    -> 2.19
    }

protected:
    /**
     * @brief: 概率值转对数概率 
     * @param prob 概率值
     */    
    float probToLogOdds(float prob) {
        float odds = prob / (1.0f - prob);
        return log(odds);
    }

    float logOddsVal;   ///< The log odds representation of occupancy probability.
    float logOddsOccupied; /// < The log odds representation of probability used for updating cells as occupied
    float logOddsFree;     /// < The log odds representation of probability used for updating cells as free
    float occ_prob_;     // 占用概率   
    bool prob_update_;  
};
}
}