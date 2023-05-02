#pragma once 

#include <cmath>

namespace msa2d {
namespace map {

class OccGridCellOperationBase {
public:
    /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
    OccGridCellOperationBase() {
    }

    /**
   * Update cell as occupied
   * @param cell The cell.
   */
    virtual void updateSetOccupied() = 0;

    /**
   * Update cell as free
   * @param cell The cell.
   */
    virtual void updateSetFree() = 0;

    virtual void updateUnsetFree() = 0;

    /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
    virtual float getGridProbability() const = 0; 

    virtual bool isOccupied() const = 0;

    virtual bool isFree() const = 0; 

    virtual void resetGridCell() = 0; 
};

class OccGridCellOperationLogOdd : public OccGridCellOperationBase {
public:
    /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
    OccGridCellOperationLogOdd() {
        logOddsVal = 0.0f;  
        // 设置free/占据 的栅格的对数概率 
        setUpdateFreeFactor(0.4f);   // 空闲的更新量     -0.42
        setUpdateOccupiedFactor(0.9f);   // 占据的更新量  0.41
    }

    /**
   * Update cell as occupied
   * @param cell The cell.
   */
    void updateSetOccupied() override {
        if (logOddsVal < 50.0f) {
            logOddsVal += logOddsOccupied;
        }
    }

    /**
   * Update cell as free
   * @param cell The cell.
   */
    void updateSetFree() override {
        logOddsVal += logOddsFree;
    }

    void updateUnsetFree() override {
        logOddsVal -= logOddsFree;
    }

    /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
    float getGridProbability() const override {
        float odds = exp(logOddsVal);
        return odds / (odds + 1.0f);
    }

    /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
    bool isOccupied() const override {
        return logOddsVal > 0.0f;   // 即概率大于0.5 认为占据
    }

    bool isFree() const override {
        return logOddsVal < 0.0f;
    }

    /**
   * Reset Cell to prior probability.
   */
    void resetGridCell() override {
        logOddsVal = 0.0f;
    }

    void setUpdateFreeFactor(float factor) {
        logOddsFree = probToLogOdds(factor);
    }

    void setUpdateOccupiedFactor(float factor) {
        logOddsOccupied = probToLogOdds(factor);
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
};

/**
 * @brief 计数法
 * 
 */
class OccGridCellOperationCnt : public OccGridCellOperationBase {
public:
    /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
    OccGridCellOperationCnt() {
        free_t_ = 10;
        occupied_t_ = 10; 
        setUpdateFreeFactor(1);
        setUpdateOccupiedFactor(1);
    }

    /**
   * Update cell as occupied
   * @param cell The cell.
   */
    void updateSetOccupied() override {
        if (occupied_t_ < 1000) {
            occupied_t_ += occupied_factor_;
        }
        if (free_t_ >= 10) {
            free_t_ -= free_factor_;  
        } 
    }

    /**
   * Update cell as free
   * @param cell The cell.
   */
    void updateSetFree() override {
        if (free_t_ < 1000) {
            free_t_ += free_factor_;
        }
        if (occupied_t_ >= 10) {
            occupied_t_ -= occupied_factor_;  
        } 
    }

    /**
     * @brief 由于占用的优先级高与free，而更新栅格时，可能出现先观测到free，后
     *                  观测到占用的情况，此时就需要将之前的free取消
     * 
     */
    void updateUnsetFree() override {
        if (occupied_t_ < 1000) {
            occupied_t_ += occupied_factor_;
        }
        if (free_t_ >= 10) {
            free_t_ -= free_factor_;  
        } 
    }

    /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
    float getGridProbability() const override {
        return static_cast<float>(occupied_t_) / (occupied_t_ + free_t_);
    }

    /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
    bool isOccupied() const override {
        return occupied_t_ > free_t_;
    }

    bool isFree() const override {
        return occupied_t_ < free_t_;
    }

    /**
   * Reset Cell to prior probability.
   */
    void resetGridCell() override {
        free_t_ = 10;
        occupied_t_ = 10; 
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