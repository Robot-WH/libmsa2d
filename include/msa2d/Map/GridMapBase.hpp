/**
 * @file GridMapBase.hpp
 * @author lwh (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-10-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 

#include <Eigen/Geometry>

namespace msa2d {
namespace map {

/**
 * @brief GridMap 的 基础类，保存map 大小，坐标等信息
 * 
 */
class GridMapBase {
public:
    /**
     * @brief 
     * @tparam _GridCellParam grid的构造参数  
     * @param {float} map_resolution
     * @param grid_size grid map 的尺寸   
     * @param map_in_world map坐标系原点在world坐标系的坐标
     * @param cell_param grid 的 构造参数 
     */    
    GridMapBase(float map_resolution, const Eigen::Vector2i& grid_size,
            const Eigen::Vector2f& map_in_world) :  lastUpdateIndex_(-1) {
        sizeX_ = grid_size[0];
        map_info_.grid_resolution = map_resolution;
        map_info_.map_grid_size = grid_size;  
        map_info_.map_world_size = {map_resolution * grid_size.x(), 
                                                                        map_resolution * grid_size.y()};
        setMapTransformation(map_in_world);
        scaleToMap_ = 1.0f / map_resolution; 
    }

    /**
     * Destructor
     */
    virtual ~GridMapBase() {
    }

    GridMapBase(const GridMapBase& other) = default;  

    const Eigen::Vector2i& getMapGridSize() const { return map_info_.map_grid_size; }
    const Eigen::Vector2f& getMapWorldSize() const { return map_info_.map_world_size; }
    int getGridSizeX() const { return map_info_.map_grid_size.x(); }
    int getGridSizeY() const { return map_info_.map_grid_size.y(); }
    int getMapAllGridNum() {return getGridSizeX() * getGridSizeY(); }

    bool pointOutOfMapBounds(const Eigen::Vector2f& coords) const {
        return ((coords[0] < 0.0f) || (coords[0] > (map_info_.map_grid_size.x() - 1)) 
            || (coords[1] < 0.0f) || (coords[1] > (map_info_.map_grid_size.y() - 1)));
    }

    bool pointOutOfMapBounds(const Eigen::Vector2i& coords) const {
        return ((coords[0] < 0) || (coords[0] > (map_info_.map_grid_size.x() - 1)) 
            || (coords[1] < 0) || (coords[1] > (map_info_.map_grid_size.y() - 1)));
    }

    bool pointOutOfMapBounds(const uint16_t& x, const uint16_t& y) const {
        return ((x < 0) || (x > (map_info_.map_grid_size.x() - 1)) 
            || (y < 0) || (y > (map_info_.map_grid_size.y() - 1)));
    }

    Eigen::Array2i GetGridIndex(const Eigen::Vector2f& pos) const {
        return Eigen::Array2i(pos.x() * scaleToMap_, pos.y() * scaleToMap_);
    }

    /**
     * Returns the world coordinates for the given map coords.
     */
    inline Eigen::Vector2f PosMapToWorldf(const Eigen::Vector2f& mapCoords) const {
        return worldTmap_ * mapCoords;
    }

    /**
     * @brief 世界坐标下的坐标转到栅格地图坐标系下
     */
    inline Eigen::Vector2f PosWorldToMapf(const Eigen::Vector2f& world_pos) const {
        // std::cout << "PosWorldToMapf, mapTworld_ * world_pos: " << (mapTworld_ * world_pos).transpose() << std::endl;
        return mapTworld_ * world_pos;
    }

    /**
     * @brief 世界坐标下的坐标转到栅格地图坐标系下
     */
    inline Eigen::Array2i PosWorldToMapi(const Eigen::Vector2f& world_pos) const {
        Eigen::Vector2f map_pos = mapTworld_ * world_pos;
        return Eigen::Array2i(map_pos.x(), map_pos.y());
    }

    /**
     * Returns the world pose for the given map pose.
     */
    inline Eigen::Vector3f PoseMapToWorld(const Eigen::Vector3f& mapPose) const {
        Eigen::Vector2f world_coords(worldTmap_ * mapPose.head<2>());
        return Eigen::Vector3f(world_coords[0], world_coords[1], mapPose[2]);
    }

    /**
     * @brief 将位姿 由相对世界坐标系 转换到相对 Map坐标系  
     * @return Tmb   相对于Map坐标系的位姿 
     */
    inline Eigen::Vector3f PoseWorldToMap(const Eigen::Vector3f& worldPose) const {
            Eigen::Vector2f mapCoords(mapTworld_ * worldPose.head<2>());   // Tmw * Twb
        return Eigen::Vector3f(mapCoords[0], mapCoords[1], worldPose[2]);
    }

    /**
     * @brief Set the Map Transformation object 获取GridMap坐标系到世界坐标系变换关系
     * @details map -> world , 先进行尺度变换，然后执行平移变换
     * @param map_in_world 
     */
    void setMapTransformation(const Eigen::Vector2f& map_in_world) {
        worldTmap_ = Eigen::Translation2f(map_in_world[0], map_in_world[1]) *
                                            Eigen::AlignedScaling2f(map_info_.grid_resolution, map_info_.grid_resolution); 
        mapTworld_ = worldTmap_.inverse();
    }

    /**
     * Returns the scale factor for one unit in world coords to one unit in map coords.
     * @return The scale factor
     */
    float getScaleToMap() const {
        return scaleToMap_;
    }

    /**
     * Returns the cell edge length of grid cells in millimeters.
     * @return the cell edge length in millimeters.
     */
    float getGridResolution() const {
        return map_info_.grid_resolution;
    }

    /**
     * Returns a reference to the homogenous 2D transform from map to world coordinates.
     * @return The homogenous 2D transform.
     */
    const Eigen::Affine2f &getWorldTmap() const {
        return worldTmap_;
    }

    /**
     * Returns a reference to the homogenous 2D transform from world to map coordinates.
     * @return The homogenous 2D transform.
     */
    const Eigen::Affine2f& getMapTworld() const {
        return mapTworld_;
    }

    void setUpdated() { lastUpdateIndex_++; };
    int getUpdateIndex() const { return lastUpdateIndex_; };

    virtual void reset() = 0;  /// 地图重置
    virtual void moveTo(const Eigen::Vector2f& new_map_pos_in_world) = 0;     /// 将地图移动

protected:
    struct Info {
        Eigen::Vector2i map_grid_size{0, 0};    // 地图 xy方向 grid的数量   即维度
        Eigen::Vector2f map_world_size{0.f, 0.f};     // 地图在真实世界的尺度 
        float grid_resolution = 0;   
    } map_info_;

    float scaleToMap_; ///< Scaling factor from world to map.

    Eigen::Affine2f worldTmap_;   ///< Homogenous 2D transform from map to world coordinates.
    Eigen::Affine2f mapTworld_;   ///< Homogenous 2D transform from world to map coordinates.

    int sizeX_;

private:
    int lastUpdateIndex_;
};
}
}
