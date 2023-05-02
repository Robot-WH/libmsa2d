/**
 * @file OccGridMapOperation.hpp
 * @author lwh (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-10-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include <mutex>
#include "OccGridMapBase.hpp"
#include "../common/UtilFunctions.hpp"

namespace msa2d {
namespace map {

/**
 * @brief 占用栅格地图操作,包含更新、同步、移动等
 * 
 */
class OccGridMapOperation {
public:
    // OccGridMapOperation(GridMap* gridMapIn, OccGridMapUtilConfig<GridMap>* gridMapUtilIn)
    OccGridMapOperation(OccGridMap* gridMapIn)
        : gridMap_(gridMapIn), mapModifyMutex_(new std::mutex()), 
            grid_occ_mark_(2), grid_free_mark_(1) {
    }
    
    virtual ~OccGridMapOperation() {
    }

    // 声明了析构则移动被抑制  所以要手动生成移动
    OccGridMapOperation(OccGridMapOperation&& other) {
        gridMap_ = other.gridMap_;
        other.gridMap_ = nullptr;
        mapModifyMutex_ = other.mapModifyMutex_; 
        other.mapModifyMutex_ = nullptr;
        grid_free_mark_ = 1; 
        grid_occ_mark_ = 2;
    }

    // 定义移动后，拷贝被自动删除了 
    // OccGridMapOperation(const OccGridMapOperation& other) = delete; 

    void cleanup() {
        delete gridMap_;
        delete mapModifyMutex_;  
    }

    // 重置地图
    void reset() {
        gridMap_->clear();
    }

    // // 重置cache中的数据
    // void resetCachedData() {
    //     gridMapUtil->resetCachedData();
    // }

    //// 获取本图层的尺度 scale = 1.0 / map_resolution.
    float getScaleToMap() const { return gridMap_->getScaleToMap(); };

    const OccGridMap& getGridMap() const { return *gridMap_; };
    OccGridMap& getGridMap() { return *gridMap_; };

    // OccGridMapUtilConfig<GridMap>& getOccGridMapUtil1() {return *gridMapUtil;};

    /// 获取当前地图的互斥锁
    std::mutex* getMapMutex() {
        return mapModifyMutex_;
    }

    /**
     * @brief 有Scan数据更新地图
     * @param laser_same_scale   当前scan激光数据 注意该激光已经进行尺度转换了，和当前GridMap栅格坐标尺度相同
     * @param scan_pose_in_world  当前scan世界系下位姿
     */
    void updateByScan(const std::vector<Eigen::Vector2f>& laser_same_scale, const Eigen::Vector3f& scan_pose_in_world) {
        mapModifyMutex_->lock(); //加锁，禁止其他线程竞争地图资源
        /// 更新地图
        std::vector<uint8_t> grid_update_marksheet(gridMap_->getMapGridSize(), 0);     // 为了保证 每一个grid只被更新一次，因此设置一个mark
        // 世界坐标转换到该栅格坐标   robot -> grid map
        Eigen::Vector3f mapPose(gridMap_->getMapCoordsPose(scan_pose_in_world));
        Eigen::Isometry2f poseTransform(
            Eigen::Translation2f(mapPose[0], mapPose[1]) * Eigen::Rotation2Df(mapPose[2]));
        Eigen::Vector2f scanBeginMapf(poseTransform * Eigen::Vector2f{0, 0});    // T(map<-laser) * t(laser) = t(map)
        // 栅格坐标是整型  因此四舍五入   地图的原点建立在grid的中间
        Eigen::Vector2i scanBeginMapi(scanBeginMapf[0] + 0.5f, scanBeginMapf[1] + 0.5f);
        int numValidElems = laser_same_scale.size();
        // std::cout << " num: " << numValidElems << "\n";
        for (int i = 0; i < numValidElems; ++i) {
            //Get map coordinates of current beam endpoint    laser_same_scale 中的激光数据已经转换到了base坐标
            Eigen::Vector2f scanEndMapf(poseTransform * laser_same_scale[i]);
            Eigen::Vector2i scanEndMapi(scanEndMapf[0] + 0.5f, scanEndMapf[1] + 0.5f);
      
            if (scanBeginMapi != scanEndMapi) {
                updateLineBresenhami(scanBeginMapi, scanEndMapi, grid_update_marksheet);
            }
        }

        gridMap_->setUpdated();
        mapModifyMutex_->unlock(); //地图解锁
    }

    /**
     * @brief: 将当前GridMap的坐标原点移动到new_map_pos_in_world
     */    
    void moveTo(const Eigen::Vector2f& new_map_pos_in_world) {
        Eigen::Vector2i pos_in_prev_map = gridMap_->getMapCoords(new_map_pos_in_world).cast<int>();  
        // std::cout << "pos_in_prev_map: " << pos_in_prev_map.transpose() << std::endl;
        auto new_map = gridMap_->createSameSizeMap(); 
        // 遍历原Map的每一个Grid
        for (uint16_t i = 0; i < gridMap_->getSizeX(); ++i) {
            for (uint16_t j = 0; j < gridMap_->getSizeY(); ++j) {
                // 计算该Grid在移动后的地图的Grid坐标
                Eigen::Vector2i new_grid_pos{i - pos_in_prev_map[0], j - pos_in_prev_map[1]};
                // 是否在该地图范围内
                if (!gridMap_->pointOutOfMapBounds(new_grid_pos)) {
                    new_map[new_grid_pos[1] * gridMap_->getSizeX() + new_grid_pos[0]] = gridMap_->getCell(i, j);
                }
            }
        }
        gridMap_->resetArray(new_map);
        // 对新地图原点在旧地图的坐标进行了取整   使得新地图与旧地图的栅格完全重合
        Eigen::Vector2f new_map_adjust_pos_in_world = 
            gridMap_->getWorldCoords(Eigen::Vector2f{pos_in_prev_map[0], pos_in_prev_map[1]});  
        gridMap_->setMapTransformation(new_map_adjust_pos_in_world); 
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
    inline void updateLineBresenhami(const Eigen::Vector2i& beginMap, 
                                                                            const Eigen::Vector2i& endMap, 
                                                                            std::vector<uint8_t>& grid_update_marksheet,
                                                                            unsigned int max_length = UINT_MAX) {
        int x0 = beginMap[0];
        int y0 = beginMap[1];

        //check if beam start point is inside map, cancel update if this is not the case
        if ((x0 < 0) || (x0 >= gridMap_->getSizeX()) || (y0 < 0) || (y0 >= gridMap_->getSizeY())) {
            return;
        }

        int x1 = endMap[0];
        int y1 = endMap[1];

        //check if beam end point is inside map, cancel update if this is not the case
        if ((x1 < 0) || (x1 >= gridMap_->getSizeX()) || (y1 < 0) || (y1 >= gridMap_->getSizeY())) {
            return;
        }

        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = math::sign(dx);
        int offset_dy = math::sign(dy) * gridMap_->getSizeX();    

        unsigned int startOffset = beginMap.y() * gridMap_->getSizeX() + beginMap.x();

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
        unsigned int endOffset = endMap.y() * gridMap_->getSizeX() + endMap.x();
        bresenhamCellOcc(endOffset, grid_update_marksheet);
    }

    // 进行bresenham画线
    inline void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b, 
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
            gridMap_->updateSetFree(index); 
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
                gridMap_->updateUnsetFree(index);
            }
            gridMap_->updateSetOccupied(index);
            grid_update_marksheet[index] = grid_occ_mark_;
        }
    }

private:
    OccGridMap* gridMap_;                                        // 地图网格
    // OccGridMapUtilConfig<GridMap>* gridMapUtil;      // 网格工具、设置
    std::mutex* mapModifyMutex_ = nullptr;              // 地图锁，修改地图时，需要加锁避免多线程资源访问冲突。
    int grid_occ_mark_;
    int grid_free_mark_;
};
}
}