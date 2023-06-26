
#pragma once
#include <deque>
#include <mutex>
#include <nabo/nabo.h>
#include "../Sensor/point_cloud.hpp"

namespace msa2d {
namespace map {

class PointcloudLocalMap {
public:
    struct  Option {
        int window_size_ = 10; 
    };
    PointcloudLocalMap() = default; 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PointcloudLocalMap(Option option);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于发生了充足的运动，因此添加一帧数据到Local map   
     */            
    void UpdateLocalMapForMotion(std::vector<Eigen::Vector2f>& curr_points);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于长时间没有更新地图，因此添加一帧数据到Local map   
     * @details 此时直接将滑动窗口最末尾的数据移除
     */            
    void UpdateLocalMapForTime(std::vector<Eigen::Vector2f>& curr_points);
    
    /**
     * @brief: 找到 point 的近邻点
     * @param name 点的标识名
     * @param num 需要的近邻点数量
     * @param max_range 近邻点的最大距离
     * @param[out] res 查找的结果 
     * @return 是否搜索到 num 个点 
     */            
    virtual bool GetNearlyNeighbor(sensor::LaserPoint const& point, uint16_t const& num, 
                                                                        double const& max_range, std::vector<sensor::LaserPoint>& res) const;

    const std::vector<Eigen::Vector2f>& ReadLocalMap() const;

protected:
    void localMapDownsampling(const std::deque<std::vector<Eigen::Vector2f>>& sliding_window);

    void createKDTreeFromLocalMap();

private:
    Option option_; 
    // kdtree内的点云数据
    Eigen::MatrixXd target_kdtree_database_;
    // kdtree 
    Nabo::NNSearchD* target_kdtree_;
    std::deque<std::vector<Eigen::Vector2f>> sliding_window_;
    std::vector<Eigen::Vector2f> localmap_;
    std::mutex local_map_mt_; 
}; // class PointcloudLocalMap
}
}

