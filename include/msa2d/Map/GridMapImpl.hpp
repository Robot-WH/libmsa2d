/**
 * @file GridMapImpl.hpp
 * @author lwh ()
 * @brief 
 * @version 0.1
 * @date 2022-10-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include "../common/tic_toc.h"
#include "GridMapBase.hpp"

namespace msa2d {
namespace map {

/**
 * @brief 栅格地图的实现  保存真正的地图的数据
 * 
 * @tparam _CellType 
 */
template <typename _CellType>
class GridMapImpl : public GridMapBase {
public:
    /**
     * @brief 
     * @tparam _GridCellParam grid的构造参数  
     * @param {float} map_resolution
     * @param size grid map 的尺寸   
     * @param map_in_world map坐标系原点在world坐标系的坐标
     * @param cell_param grid 的 构造参数 
     */    
    GridMapImpl(float map_resolution, const Eigen::Vector2i& size,
            const Eigen::Vector2f& map_in_world) : GridMapBase(map_resolution, size, map_in_world) {
        allocateArray(size);
    }

    // GridMapBase(const GridMapBase& other) = delete; 
    // GridMapBase& operator=(const GridMapBase& other) = delete; 
    /**
     * @brief 拷贝构造函数 
     * @param other 
     */
    GridMapImpl(const GridMapImpl& other) : GridMapBase(other) {
        /** 创建一个新的Map array，拷贝原来的数据 **/
        // 先构造内存空间
        auto size = getMapAllGridNum();
        mapArray_ = new _CellType[size];
        // 将数据复制   深拷贝 
        for (int i = 0; i < size; ++i) {
            mapArray_[i] = other.mapArray_[i];   // 拷贝赋值  
        }
    }

    /**
     * @brief 拷贝赋值 
     * 
     * @param other 
     * @return GridMapBase& 
     */
    GridMapImpl& operator=(const GridMapImpl& other) {
        if (this != &other) {
            GridMapBase::operator=(other); // 调用基类的拷贝赋值函数
             // 地图的size不同那么重新创建地图  
            if (!(map_info_.map_grid_size == other.map_info_.map_grid_size)) {
                rebuildMap(other.map_info_.map_grid_size);
            }
            // 将mapArray_数据复制   
            auto size = getMapAllGridNum();
            for (int i = 0; i < size; ++i) {
                mapArray_[i] = other.mapArray_[i];   // 拷贝赋值  
            }
        }

        return *this;
    }

    /**
     * @brief Destroy the Grid Map Impl object
     * 
     */
    virtual ~GridMapImpl() {
        deleteArray();
    }

    /**
     * @brief Create a Same Size Map object
     * 
     * @return _CellType* 
     */
    virtual _CellType* createSameSizeMap() {
        std::cout << "createSameSizeMap, getMapAllGridNum(): " << getMapAllGridNum() << std::endl;
        _CellType* mapArray = new _CellType[getMapAllGridNum()];
        std::cout << "createSameSizeMap, done " << std::endl;
        return mapArray; 
    }

    /**
     * @brief 重建地图 
     * @param new_size 
     */
    void rebuildMap(const Eigen::Vector2i& new_size) {
        if (new_size != map_info_.map_grid_size) {
            deleteArray();
            allocateArray(new_size);
            map_info_.map_grid_size = new_size;
        } else {
            clear();   // 地图的数据清空 
        }
    }

    /**
     * @brief 重置地图
     * 
     */
    void reset() {
        clear();
    }

    /**
     * @brief Resets the grid cell values by using the resetGridCell() function.
     * 
     */
    void clear() {
        // std::cout << "clear grid map " << std::endl;
        int size = getMapAllGridNum();

        for (int i = 0; i < size; ++i) {
            mapArray_[i].resetGridCell();
        }
    }

    /**
     * @brief  Allocates memory for the two dimensional pointer array for map representation.
     * 
     * @param new_size 
     */
    void allocateArray(const Eigen::Vector2i& new_size) {
        int sizeX = new_size.x();
        int sizeY = new_size.y();

        mapArray_ = new _CellType[sizeX * sizeY];
    }

    /**
     * @brief 
     * 
     */
    void deleteArray() {
        if (mapArray_ != nullptr) {
            delete[] mapArray_;
            mapArray_ = nullptr;
        }
    }

    /**
     * @brief 
     * 
     * @param new_mapArray 
     */
    void resetArray(_CellType* new_mapArray) {
        delete[] mapArray_;
        mapArray_ = new_mapArray; 
    }

    /**
     * @brief Get the Cell object
     *                    安全的Cell获取，将判断Cell的坐标是否合法
     * @param x 
     * @param y 
     * @param cell 
     * @return true 获取成功
     * @return false  获取失败
     */
    bool getCell(int x, int y, _CellType& cell) {
        if (pointOutOfMapBounds(x, y)) {
            return false;
        }
        cell = mapArray_[y * sizeX_ + x];
        return true; 
    }

    /**
     * @brief Get the Cell object
     *                  直接获取Cell,不安全，需要在外部确保坐标合法
     * @param x 
     * @param y 
     * @return _CellType& 
     */
    _CellType& getCell(int x, int y) {
        return mapArray_[y * sizeX_ + x];
    }

    /**
     * @brief Get the Cell object
     *                  直接获取Cell,不安全，需要在外部确保坐标合法
     * @param x 
     * @param y 
     * @return const _CellType& 
     */
    const _CellType& getCell(int x, int y) const {
        return mapArray_[y * sizeX_ + x];
    }

    /**
     * @brief Get the Cell object
     *                  直接获取Cell,不安全，需要在外部确保坐标合法
     * @param index 
     * @return _CellType& 
     */
    _CellType& getCell(int index) {
        return mapArray_[index];
    }

    /**
     * @brief Get the Cell object
     *                  直接获取Cell,不安全，需要在外部确保坐标合法
     * @param index 
     * @return const _CellType& 
     */
    const _CellType& getCell(int index) const {
        return mapArray_[index];
    }

    /**
     * @brief Get the Map Extends object
     *                  Returns the rectangle ([xMin,yMin],[xMax,xMax]) containing non-default cell values
     * @param xMax 
     * @param yMax 
     * @param xMin 
     * @param yMin 
     * @return true 
     * @return false 
     */
    bool getMapExtends(int &xMax, int &yMax, int &xMin, int &yMin) const {
        int lowerStart = -1;
        int upperStart = 10000;

        int xMaxTemp = lowerStart;
        int yMaxTemp = lowerStart;
        int xMinTemp = upperStart;
        int yMinTemp = upperStart;

        int sizeX = this->getSizeX();
        int sizeY = this->getSizeY();

        for (int x = 0; x < sizeX; ++x) {
            for (int y = 0; y < sizeY; ++y) {
                if (this->mapArray_[x][y].getValue() != 0.0f) {

                    if (x > xMaxTemp) {
                        xMaxTemp = x;
                    }

                    if (x < xMinTemp) {
                        xMinTemp = x;
                    }

                    if (y > yMaxTemp) {
                        yMaxTemp = y;
                    }

                    if (y < yMinTemp) {
                        yMinTemp = y;
                    }
                }
            }
        }

        if ((xMaxTemp != lowerStart) &&
            (yMaxTemp != lowerStart) &&
            (xMinTemp != upperStart) &&
            (yMinTemp != upperStart)) {
            xMax = xMaxTemp;
            yMax = yMaxTemp;
            xMin = xMinTemp;
            yMin = yMinTemp;
            return true;
        } else {
            return false;
        }
    }

    /**
     * @brief: 将当前GridMap的坐标原点移动到new_map_pos_in_world
     */    
    void moveTo(const Eigen::Vector2f& new_map_pos_in_world) override {
        // std::cout << "moveTo begin, new_map_pos_in_world: " << new_map_pos_in_world.transpose() << std::endl;
        Eigen::Vector2i pos_in_prev_map = PosWorldToMapf(new_map_pos_in_world).cast<int>();  
        std::cout << "PosWorldToMapf(), pos_in_prev_map: " << pos_in_prev_map.transpose() << std::endl;
        _CellType* new_map = createSameSizeMap(); 
        std::cout << "createSameSizeMap()" << std::endl;
        // 遍历原Map的每一个Grid
        for (uint16_t i = 0; i < getGridSizeX(); ++i) {
            for (uint16_t j = 0; j < getGridSizeY(); ++j) {
                // 计算该Grid在移动后的地图的Grid坐标
                Eigen::Vector2i new_grid_pos{i - pos_in_prev_map[0], j - pos_in_prev_map[1]};
                // 是否在该地图范围内
                if (!pointOutOfMapBounds(new_grid_pos)) {
                    //  std::cout << "i: " << i << ",j:" << j << ",new_grid_pos x: " << new_grid_pos[0] << ",y: " << new_grid_pos[1] << std::endl;
                    new_map[new_grid_pos[1] * getGridSizeX() + new_grid_pos[0]] = getCell(i, j);
                }
            }
        }
        std::cout << "for for " << std::endl;
        resetArray(new_map);   // 将 mapArray_  指向 new_map 
        // std::cout << "resetArray " << std::endl;
        // 对新地图原点在旧地图的坐标进行了取整   使得新地图与旧地图的栅格完全重合
        Eigen::Vector2f new_map_adjust_pos_in_world = 
            PosMapToWorldf(Eigen::Vector2f{pos_in_prev_map[0], pos_in_prev_map[1]});  
        setMapTransformation(new_map_adjust_pos_in_world); 
        std::cout << "finish" << std::endl;
    }

protected:
    _CellType* mapArray_; ///< Map representation used with plain pointer array.
};
}
}
