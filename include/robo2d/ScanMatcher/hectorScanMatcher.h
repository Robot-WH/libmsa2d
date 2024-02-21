#pragma once 
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include "../Sensor/point_cloud.hpp"
#include "../Map/OccGridMapBase.hpp"
#include "../Map/OccGridMapPyramid.h"
namespace robo2d {
namespace ScanMatcher {
/**
 * @brief: hector-slam所使用的匹配算法
 * @details: 特点：多地图
 */
class hectorScanMatcher {
public:
    struct Option {
        int multi_map_num_ = 3;    // 使用的地图数量
        uint8_t lowest_layer_idx_; // 使用金字塔地图的最低层序号 
        uint8_t highest_layer_idx_; // 使用金字塔地图的最高层序号 
    };
    hectorScanMatcher(Option option);
    ~hectorScanMatcher();

    /**
     * 地图匹配，通过多分辨率地图求解当前激光帧的pose
     * @param predictPoseInWorld 世界坐标下先验位姿
     * @param dataContainers 激光观测金字塔数据
     * @param map 匹配金字塔地图  
     * @param covMatrix
     * @return  
     */
    virtual Eigen::Vector3f Solve(const Eigen::Vector3f& predictPoseInWorld, 
                                                                const std::vector<sensor::LaserPosContainer>& dataContainers,
                                                                map::OccGridMapPyramid& map, 
                                                                Eigen::Matrix3f& covMatrix);

private:
    /**
     * 实际进行位姿估计的函数
     * @param predictPoseInWorld  位姿初值
     * @param grid_map  参与匹配的栅格地图 
     * @param dataContainer   激光数据    与  当前OccMap同分辨率  
     * @param covMatrix   协方差矩阵
     * @param maxIterations   最大迭代次数
     * @return
     */
    Eigen::Vector3f matchData(const Eigen::Vector3f& predictPoseInWorld, 
                                                            map::OccGridMapBase* grid_map, 
                                                            const sensor::LaserPosContainer& dataContainer,
                                                            Eigen::Matrix3f& covMatrix, 
                                                            int maxIterations);

protected:
    /**
     *  高斯牛顿估计位姿
     * @param estimate      相对于当前Map的位姿
     * @param gridMapUtil   网格地图相关计算工具
     * @param dataPoints    激光数据
     * @return  提示是否有解　－－－　貌似没用上
    */
    bool estimateTransformationGN(Eigen::Vector3f& estimate,
                                                                        map::OccGridMapBase* grid_map,
                                                                        const sensor::LaserPosContainer& dataPoints,
                                                                        bool evaluate_degenerate = false);

    void updateEstimatedPose(Eigen::Vector3f &estimate, const Eigen::Vector3f &change);

    /**
     * 使用当前pose投影dataPoints到地图，计算出 H 矩阵 b列向量， 理论部分详见Hector论文： 《A Flexible and Scalable SLAM System with Full 3D Motion Estimation》.
     * @param pose    地图系上的位姿   base -> map
     * @param grid_map 
     * @param dataPoints  已转换为地图尺度的激光点数据
     * @param H   需要计算的 H矩阵
     * @param dTr  需要计算的 g列向量
     */
    void getCompleteHessianDerivs(const Eigen::Vector3f& pose,
                                                                        map::OccGridMapBase* grid_map,
                                                                        const sensor::LaserPosContainer& dataPoints,
                                                                        Eigen::Matrix3f& H,
                                                                        Eigen::Vector3f& dTr);

    /**
     * 双线性插值计算网格中任一点的得分（占据概率）以及该点处的梯度
     * @param coords  激光点地图坐标
     * @return ret(0) 是网格值 ， ret(1) 是栅格值在x方向的导数 ， ret(2)是栅格值在y方向的导数
     */
    Eigen::Vector3f interpMapValueWithDerivatives(map::OccGridMapBase* grid_map, 
                                                                                                        const Eigen::Vector2f& coords);

protected:
    Option option_;  
    Eigen::Vector3f dTr_;
    Eigen::Matrix3f H_;
    Eigen::Matrix3f V_u_;  
    Eigen::Matrix3f V_f_;  
    bool is_degenerate_ = false;
};
}
}

