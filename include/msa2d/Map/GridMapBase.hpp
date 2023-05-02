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

#include <Eigen/Geometry>
#include <Eigen/LU>

namespace msa2d {
namespace map {
/**
 * GridMapBase provides basic grid map functionality (creates grid , provides transformation from/to world coordinates).
 * It serves as the base class for different map representations that may extend it's functionality.
 */
template <typename _CellType>
class GridMapBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @param {float} map_resolution
     * @param size grid map 的尺寸   
     * @param map_in_world map坐标系原点在world坐标系的坐标
     */    
    template<typename ...CellParam>
    GridMapBase(float map_resolution, const Eigen::Vector2i& size,
            const Eigen::Vector2f& map_in_world, CellParam... cell_param) 
            : mapArray_(nullptr), lastUpdateIndex_(-1) {
        sizeX_ = size[0];
        map_info_.grid_resolution = map_resolution;
        rebuildMap(size);
        setMapTransformation(map_in_world);
        scaleToMap_ = 1.0f / map_resolution; 
    }

    /**
     * Copy Constructor, only needed if pointer members are present.
     */
    GridMapBase(const GridMapBase& other) {
        allocateArray(other.getMapGridSize());
        *this = other;
    }

    /**
     * Assignment operator, only needed if pointer members are present.
     */
    GridMapBase& operator=(const GridMapBase& other) {
        if (!(map_info_.map_grid_size == other.map_info_.map_grid_size)) {
            rebuildMap(other.map_info_.map_grid_size);
        }

        map_info_ = other.map_info_; 
        worldTmap_ = other.worldTmap_;
        mapTworld_ = other.mapTworld_;
        scaleToMap_ = other.scaleToMap_;
        //@todo potential resize
        int sizeX = getSizeX();
        int sizeY = getSizeY();
        size_t concreteCellSize = sizeof(_CellType);
        memcpy(mapArray_, other.mapArray_, sizeX * sizeY * concreteCellSize);

        return *this;
    }

    /**
     * Destructor
     */
    virtual ~GridMapBase() {
        deleteArray();
    }

    const Eigen::Vector2i& getMapGridSize() const { return map_info_.map_grid_size; }
    int getSizeX() const { return map_info_.map_grid_size.x(); }
    int getSizeY() const { return map_info_.map_grid_size.y(); }
    int getMapGridSize() {return getSizeX() * getSizeY(); }

    _CellType* createSameSizeMap() {
        _CellType* mapArray = new _CellType[getMapGridSize()];
        return mapArray; 
    }

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

    /**
     * Resets the grid cell values by using the resetGridCell() function.
     */
    void clear() {
        int size = getSizeX() * getSizeY();

        for (int i = 0; i < size; ++i) {
            mapArray_[i].resetGridCell();
        }
    }

    /**
     * Allocates memory for the two dimensional pointer array for map representation.
     */
    template<typename ...CellParam>
    void allocateArray(const Eigen::Vector2i& new_size, CellParam... cell_param) {
        int sizeX = new_size.x();
        int sizeY = new_size.y();

        mapArray_ = new _CellType[sizeX * sizeY](cell_param...);
    }

    void deleteArray() {
        if (mapArray_ != nullptr) {
            delete[] mapArray_;
            mapArray_ = nullptr;
        }
    }

    void resetArray(_CellType* new_mapArray) {
        delete[] mapArray_;
        mapArray_ = new_mapArray; 
    }

    bool getCell(int x, int y, _CellType& cell) {
        if (pointOutOfMapBounds(x, y)) {
            return false;
        }
        cell = mapArray_[y * sizeX_ + x];
        return true; 
    }

    _CellType& getCell(int x, int y) {
        return mapArray_[y * sizeX_ + x];
    }

    const _CellType& getCell(int x, int y) const {
        return mapArray_[y * sizeX_ + x];
    }

    _CellType& getCell(int index) {
        return mapArray_[index];
    }

    const _CellType& getCell(int index) const {
        return mapArray_[index];
    }

    void rebuildMap(const Eigen::Vector2i& new_size) {
        if (new_size != map_info_.map_grid_size) {
            deleteArray();
            allocateArray(new_size);
            map_info_.map_grid_size = new_size;
        }
        clear();   // 地图的数据清空 
    }

    /**
     * Returns the world coordinates for the given map coords.
     */
    inline Eigen::Vector2f getWorldCoords(const Eigen::Vector2f& mapCoords) const {
        return worldTmap_ * mapCoords;
    }

    /**
     * @brief 世界坐标下的坐标转到栅格地图坐标系下
     */
    inline Eigen::Vector2f getMapCoords(const Eigen::Vector2f& world_coords) const {
        return mapTworld_ * world_coords;
    }

    /**
     * Returns the world pose for the given map pose.
     */
    inline Eigen::Vector3f getWorldCoordsPose(const Eigen::Vector3f& mapPose) const {
        Eigen::Vector2f world_coords(worldTmap_ * mapPose.head<2>());
        return Eigen::Vector3f(world_coords[0], world_coords[1], mapPose[2]);
    }

    /**
     * @brief 将位姿 由相对世界坐标系 转换到相对 Map坐标系  
     * @return Tmb   相对于Map坐标系的位姿 
     */
    inline Eigen::Vector3f getMapCoordsPose(const Eigen::Vector3f& worldPose) const {
         Eigen::Vector2f mapCoords(mapTworld_ * worldPose.head<2>());   // Tmw * Twb
        return Eigen::Vector3f(mapCoords[0], mapCoords[1], worldPose[2]);
    }

    void setMapTransformation(const Eigen::Vector2f& map_in_world) {
        worldTmap_ = Eigen::AlignedScaling2f(map_info_.grid_resolution, map_info_.grid_resolution) 
                                        * Eigen::Translation2f(map_in_world[0] / map_info_.grid_resolution, 
                                                                                    map_in_world[1] / map_info_.grid_resolution);
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
    float getCellLength() const {
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

    /**
    * Returns the rectangle ([xMin,yMin],[xMax,xMax]) containing non-default cell values
    */
    bool getMapExtends(int &xMax, int &yMax, int &xMin, int &yMin) const {
        int lowerStart = -1;
        int upperStart = 10000;

        int xMaxTemp = lowerStart;
        int yMaxTemp = lowerStart;
        int xMinTemp = upperStart;
        int yMinTemp = upperStart;

        int sizeX = getSizeX();
        int sizeY = getSizeY();

        for (int x = 0; x < sizeX; ++x)
        {
            for (int y = 0; y < sizeY; ++y)
            {
                if (this->mapArray_[x][y].getValue() != 0.0f)
                {

                    if (x > xMaxTemp)
                    {
                        xMaxTemp = x;
                    }

                    if (x < xMinTemp)
                    {
                        xMinTemp = x;
                    }

                    if (y > yMaxTemp)
                    {
                        yMaxTemp = y;
                    }

                    if (y < yMinTemp)
                    {
                        yMinTemp = y;
                    }
                }
            }
        }

        if ((xMaxTemp != lowerStart) &&
            (yMaxTemp != lowerStart) &&
            (xMinTemp != upperStart) &&
            (yMinTemp != upperStart))
        {
            xMax = xMaxTemp;
            yMax = yMaxTemp;
            xMin = xMinTemp;
            yMin = yMinTemp;
            return true;
        }
        else
        {
            return false;
        }
    }

protected:
    struct Info {
        Eigen::Vector2i map_grid_size{0, 0};    // 地图 xy方向 grid的数量   即维度  
        float grid_resolution = 0; 
    } map_info_;
    _CellType* mapArray_; ///< Map representation used with plain pointer array.

    float scaleToMap_; ///< Scaling factor from world to map.

    Eigen::Affine2f worldTmap_;   ///< Homogenous 2D transform from map to world coordinates.
    Eigen::Affine2f mapTworld_;   ///< Homogenous 2D transform from world to map coordinates.

    int sizeX_;

private:
    int lastUpdateIndex_;
};
}
}
