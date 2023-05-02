//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once 
#include <iostream>
#include "OccGridMapOperation.hpp"
#include "OccGridMapBase.hpp"
#include "../common/color.hpp"

namespace msa2d {
namespace map {

/**
 * 金字塔（多分辨率）栅格地图对象
 */
class GridMapPyramid {
public:
    struct Option {
        float bottom_resolution;
        int map_sizeX;
        int map_sizeY;
        unsigned int num_depth;   // 金字塔的层数 
        float min_distance_to_boundary;
    }; 
    /**
     * 构建金字塔地图，第一层地图格子代表的物理尺寸最小，格子数最多，地图精度高；
     * 层数越高，格子代表的物理尺寸越大，格子数越少，精度越低。
     */
    GridMapPyramid(const Option& option);

    virtual ~GridMapPyramid();

    virtual void reset();

    float getScaleToMap() const;// 获取地图尺度 scale = 1.0 / map_resolution.

    int getMapLevels() const;                       // 获取地图总层数

    const OccGridMap& getGridMap(int mapLevel) const;// 获取指定层的地图

    /// 获取指定层的地图锁
    std::mutex* getMapMutex(int i);

    /// 提示地图已经得到更新，cache中的临时数据无效
    void onMapUpdated();

    /**
     * 每层地图由当前scan与计算的位姿进行更新
     * @param dataContainer    第一层激光数据，其他层激光数据存在 dataContainers 中
     * @param laser_pose_in_world   当前帧的世界系下位姿
     */
    void updateByScan(const std::vector<sensor::LaserPointContainer>& data_containers, 
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
    std::vector<OccGridMapOperation> mapOperateContainer_; /// 不同图层的地图操作对象    层数越高  分辨率越低   
    std::vector<sensor::LaserPointContainer> dataContainers;  /// 不同图层对应的激光数据
};
} // namespace 
}