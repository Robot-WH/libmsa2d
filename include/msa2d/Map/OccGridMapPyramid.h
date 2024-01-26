/**
 * @file OccGridMapPyramid.h
 * @author lwh (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-10-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include <iostream>
#include "OccGridMapImpl.hpp"
#include "../common/color.hpp"

namespace msa2d {
namespace map {

/**
 * 金字塔（多分辨率）占用栅格地图对象
 */
class OccGridMapPyramid {
public:
    struct Option {
        float bottom_resolution;
        int map_sizeX;
        int map_sizeY;
        unsigned int num_depth;   // 金字塔的层数 
        float min_distance_to_boundary;
        std::string grid_update_method;   // 栅格更新方法 
    }; 
    /**
     * 构建金字塔地图，第一层地图格子代表的物理尺寸最小，格子数最多，地图精度高；
     * 层数越高，格子代表的物理尺寸越大，格子数越少，精度越低。
     */
    OccGridMapPyramid(const Option& option);

    virtual ~OccGridMapPyramid();

    void reset();

    float getScaleToMap() const;// 获取地图尺度 scale = 1.0 / map_resolution.

    int getMapLevels() const;                       // 获取地图总层数

    OccGridMapBase* getGridMap(int mapLevel);// 获取指定层的地图

    /// 获取指定层的地图锁
    std::mutex* getMapMutex(int i);

    /// 提示地图已经得到更新，cache中的临时数据无效
    void onMapUpdated();

    /**
     * 每层地图由当前scan与计算的位姿进行更新
     * @param dataContainer    第一层激光数据，其他层激光数据存在 dataContainers 中
     * @param laser_pose_in_world   当前帧的世界系下位姿
     */
    void updateByScan(const std::vector<sensor::LaserPosContainer>& data_containers,
                                            const std::vector<sensor::LaserPosContainer>& invalid_data_containers,
                                            const Eigen::Vector3f &laser_pose_in_world);

    /**
     * @brief: 检查是否接近地图金字塔边界  
     * @param {Vector3f} &laser_pose_in_world
     * @return {*}
     */    
    bool isCloseToBoundary(const Eigen::Vector3f& laser_pose_in_world);

    /**
     * @brief: 将地图的移动到世界坐标系上一个新的位置
     * @param 
     * @return {*}
     */    
    void moveTo(const Eigen::Vector2f& new_map_pos_in_world);

    /** 设置网格为free状态的概率 **/
    void setUpdateFactorFree(float free_factor);
    
    /** 设置网格为occupied状态的概率 **/
    void setUpdateFactorOccupied(float occupied_factor);

protected:
    Option option_; 
    Eigen::Vector2f map_in_world_;    // 地图原点在世界坐标系下的坐标
    std::vector<OccGridMapBase*> OccGridMapContainer_; /// 不同图层的地图操作对象    层数越高  分辨率越低   
};
} // namespace 
}
