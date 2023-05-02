
#pragma once 

#include <vector>

namespace msa2d {
namespace sensor {

class LaserPointContainer {
public:
    LaserPointContainer(int size = 1000) {
        dataPoints.reserve(size);
    }

    void setFrom(const LaserPointContainer &other, float factor) {
        origo = other.getOrigo() * factor;
        dataPoints = other.dataPoints;
        unsigned int size = dataPoints.size();

        for (unsigned int i = 0; i < size; ++i) {
            dataPoints[i] *= factor;
        }
    }

    void reserve(const int &size) {
        dataPoints.reserve(size);
    }

    void add(const Eigen::Vector2f &dataPoint) {
        dataPoints.push_back(dataPoint);
    }

    void clear() {
        dataPoints.clear();
    }

    int getSize() const {
        return dataPoints.size();
    }

    const Eigen::Vector2f &getVecEntry(int index) const {
        return dataPoints[index];
    }

    Eigen::Vector2f getOrigo() const {
        return origo;
    }

    void setOrigo(const Eigen::Vector2f &origoIn) {
        origo = origoIn;
    }
    std::vector<Eigen::Vector2f> dataPoints;
    Eigen::Vector2f origo{0, 0};    // 实际激光雷达的观测位置  laser到base的位置外参  
};
} // namespace 
}

