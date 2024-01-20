/**
 * @file OccGridMapBase.hpp
 * @author lwh(you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-10-02
 * 
 * @copyright Copyright (c) 2022
 */
#pragma once 
#include "OccGridMapBase.hpp"
namespace msa2d {
namespace map {
/**
 * @brief Occupancy grid map 
 * 
 */
template<typename _OccGridType>
class OccGridMapImpl : public OccGridMapBase {
public:
    /**
     * @param map_resolution 地图的分辨率 
     * @param grid_size 地图的栅格size   
     * @param map_in_world map坐标系原点在world坐标系的坐标
     */    
    OccGridMapImpl(float map_resolution, const Eigen::Vector2i& grid_size, 
                                        const Eigen::Vector2f& map_in_world)
        : occ_grid_map_(map_resolution, grid_size, map_in_world),
            grid_occ_mark_(2), grid_free_mark_(1)  {}

    virtual ~OccGridMapImpl() {}

    /**
     * @brief 
     * 
     * @param index 
     */
    void updateSetOccupied(int index) override {
        occ_grid_map_.getCell(index).updateSetOccupied();
    }

    void updateSetFree(int index) override {
        occ_grid_map_.getCell(index).updateSetFree();
    }

    void updateUnsetFree(int index) override {
        occ_grid_map_.getCell(index).updateUnsetFree();
    }

    const float& getGridProbability(const int& index) override {
        return occ_grid_map_.getCell(index).getGridProbability();
    }

    const float& getGridProbability(const int& xMap, const int& yMap) override {
        return occ_grid_map_.getCell(xMap, yMap).getGridProbability();
    }

    bool isOccupied(int xMap, int yMap) const override {
        if (occ_grid_map_.pointOutOfMapBounds(xMap, yMap)) {
            return false;
        }
        return occ_grid_map_.getCell(xMap, yMap).isOccupied();
    }

    bool isFree(int xMap, int yMap) const override {
        if (occ_grid_map_.pointOutOfMapBounds(xMap, yMap)) {
            return false;
        }
        return occ_grid_map_.getCell(xMap, yMap).isFree();
    }

    bool isOccupied(int index) const override {
        return occ_grid_map_.getCell(index).isOccupied();
    }

    bool isFree(int index) const override {
        return occ_grid_map_.getCell(index).isFree();
    }

    bool isUnknow(int xMap, int yMap) const override {
        return occ_grid_map_.getCell(xMap, yMap).isUnknow();
    }

    bool isUnknow(int index) const override {
        return occ_grid_map_.getCell(index).isUnknow();
    }

    void reset() override {
        occ_grid_map_.reset();   
    }

    void clear() override {
        occ_grid_map_.clear();  
    }

    void moveTo(const Eigen::Vector2f& new_map_pos_in_world) override {
        std::cout << "OccGridMapImpl moveTo" << std::endl;
        occ_grid_map_.moveTo(new_map_pos_in_world);
        std::cout << "OccGridMapImpl moveTo end" << std::endl;
    }

    const GridMapBase& getGridMapBase() const override {
        return occ_grid_map_;
    }

    /**
     * @brief 有Scan数据更新地图
     * @param laser_same_scale   当前scan激光数据 注意该激光已经进行尺度转换了，和当前GridMap栅格坐标尺度相同
     * @param scan_pose_in_world  当前scan世界系下位姿
     */
    void updateByScan(const std::vector<Eigen::Vector2f>& laser_same_scale, 
            const Eigen::Vector3f& scan_pose_in_world) override {
        mapModifyMutex_->lock(); //加锁，禁止其他线程竞争地图资源
        /// 更新地图
        std::vector<uint8_t> grid_update_marksheet(occ_grid_map_.getMapAllGridNum(), 0);     // 为了保证 每一个grid只被更新一次，因此设置一个mark
        // 世界坐标转换到该栅格坐标   robot -> grid map
        Eigen::Vector3f mapPose(occ_grid_map_.PoseWorldToMap(scan_pose_in_world));
        Eigen::Isometry2f poseTransform(
            Eigen::Translation2f(mapPose[0], mapPose[1]) * Eigen::Rotation2Df(mapPose[2]));
        Eigen::Vector2f scanBeginMapf(poseTransform * Eigen::Vector2f{0, 0});    // T(map<-laser) * t(laser) = t(map)
        // 当前帧起始的位置
        Eigen::Vector2i scanBeginMapi(scanBeginMapf[0], scanBeginMapf[1]);
        int numValidElems = laser_same_scale.size();

        for (int i = 0; i < numValidElems; ++i) {
            //Get map coordinates of current beam endpoint    laser_same_scale 中的激光数据已经转换到了base坐标
            Eigen::Vector2f scanEndMapf(poseTransform * laser_same_scale[i]);
            Eigen::Vector2i scanEndMapi(scanEndMapf[0], scanEndMapf[1]);
      
            if (scanBeginMapi != scanEndMapi) {
                updateLineBresenhami(scanBeginMapi, scanEndMapi, grid_update_marksheet);
            }
        }

        occ_grid_map_.setUpdated();
        mapModifyMutex_->unlock(); //地图解锁
    }

protected:
    /**
     * @brief: Bresenhami画线算法更新map
     * @param beginMap 画线的地图系起始坐标  
     * @param endMap 画线的地图系末端坐标  
     * @param grid_update_marksheet 栅格的更新标记表
     * @param max_length 更新的最大长度  
     * @return {*}
     */    
    void updateLineBresenhami(const Eigen::Vector2i& beginMap, 
                                                                    const Eigen::Vector2i& endMap, 
                                                                    std::vector<uint8_t>& grid_update_marksheet,
                                                                    unsigned int max_length = UINT_MAX) {
        int x0 = beginMap[0];
        int y0 = beginMap[1];

        //check if beam start point is inside map, cancel update if this is not the case
        if ((x0 < 0) || (x0 >= occ_grid_map_.getGridSizeX()) || (y0 < 0) || (y0 >= occ_grid_map_.getGridSizeY())) {
            return;
        }

        int x1 = endMap[0];
        int y1 = endMap[1];

        //check if beam end point is inside map, cancel update if this is not the case
        if ((x1 < 0) || (x1 >= occ_grid_map_.getGridSizeX()) || (y1 < 0) || (y1 >= occ_grid_map_.getGridSizeY())) {
            return;
        }

        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = math::sign(dx);
        int offset_dy = math::sign(dy) * occ_grid_map_.getGridSizeX();    

        unsigned int startOffset = beginMap.y() * occ_grid_map_.getGridSizeX() + beginMap.x();

        //if x is dominant
        if (abs_dx >= abs_dy) {
            int error_y = abs_dx / 2;
            bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, startOffset, grid_update_marksheet);
        } else {
            //otherwise y is dominant
            int error_x = abs_dy / 2;
            bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, startOffset, grid_update_marksheet);
        }
        // 将终点单独拿出来，设置占用
        unsigned int endOffset = endMap.y() * occ_grid_map_.getGridSizeX() + endMap.x();
        bresenhamCellOcc(endOffset, grid_update_marksheet);
    }

    // 进行bresenham画线
    void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, 
            int offset_a, int offset_b, unsigned int offset, std::vector<uint8_t>& grid_update_marksheet) {
        // https://www.jianshu.com/p/d63bf63a0e28
        // 先把起点格子设置成free
        bresenhamCellFree(offset, grid_update_marksheet);
        unsigned int end = abs_da - 1;

        for (unsigned int i = 0; i < end; ++i) {
            offset += offset_a;
            // 对应 Sub += dy/dx, 这里的实现是对 左右两边同乘 dx 后的结果
            error_b += abs_db;  
            // 判断 Sub > 0 
            if ((unsigned int)error_b >= abs_da) {
                offset += offset_b;
                // 对应Sub += dy/dx - 1, dy/dx 在之前加过了，所以这里只减 1 ，再左右两边同乘 dx
                error_b -= abs_da;  
            }
            // 再将路径上的其他点设置成free
            bresenhamCellFree(offset, grid_update_marksheet);
        }
    }

    /**
     * @brief 更新这个格子为空闲格子，只更新这一个格子
     *                  占用的优先级高与free，即只要该栅格存在占据的观测，则free无效  
     * @param index 
     * @param grid_update_marksheet 
     */
    inline void bresenhamCellFree(unsigned int index, std::vector<uint8_t>& grid_update_marksheet) {
        // 每一轮画线，每个格子只更新一次free
        // grid_update_marksheet 初始化为0 ，某个栅格占用则设置为 grid_occ_mark_ = 2 ，free 则 grid_free_mark_ = 1
        if (grid_update_marksheet[index] < grid_free_mark_) {   
            updateSetFree(index); 
            grid_update_marksheet[index] = grid_free_mark_;
        }
    }

    /**
     * @brief 更新这个格子为占用格子，只更新这一个格子
     *                  占用的优先级高与free，即只要该栅格存在占据的观测，则free无效 
     * @param index 
     * @param grid_update_marksheet 
     */
    inline void bresenhamCellOcc(unsigned int index, std::vector<uint8_t>& grid_update_marksheet) {
        // 每一轮画线，每个格子只更新一次占用
        if (grid_update_marksheet[index] < grid_occ_mark_) {
            // 如果这个格子被设置成free了，先取消free，再设置占用
            if (grid_update_marksheet[index] == grid_free_mark_) {
                updateUnsetFree(index);
            }
            updateSetOccupied(index);
            grid_update_marksheet[index] = grid_occ_mark_;
        }
    }

private:
    GridMapImpl<_OccGridType> occ_grid_map_;  
    int grid_occ_mark_;
    int grid_free_mark_;
};
} // namespace Estimator2D
}
