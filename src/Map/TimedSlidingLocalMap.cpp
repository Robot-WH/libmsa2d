#include <glog/logging.h>
#include "robo2d/Map/TimedSlidingLocalMap.h"
namespace robo2d {
namespace map {
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointcloudLocalMap::PointcloudLocalMap(Option option) : option_(option) {
    LOG(INFO) << "create PointcloudLocalMap, window size:" << option_.window_size_;
    localmap_.reserve(option_.window_size_ * 1000); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointcloudLocalMap::UpdateLocalMapForMotion(std::vector<Eigen::Vector2f>& curr_points) {
    if (curr_points.empty()) return;  
    //TicToc tt;
    // 更新滑动窗口      0.1ms   
    if (sliding_window_.size() >= option_.window_size_) {  
        sliding_window_.pop_front();
        sliding_window_.push_back(std::move(curr_points));
    } else {  
        sliding_window_.push_back(std::move(curr_points));   
    }
    // 进行降采样滤波
    localMapDownsampling(sliding_window_); 
    // tt.toc("update localmap points ");
    return;  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointcloudLocalMap::UpdateLocalMapForTime(std::vector<Eigen::Vector2f>& curr_points) {
    if (curr_points.empty()) return;  
    if (sliding_window_.size() >= option_.window_size_) {
        sliding_window_.pop_back();
    }
    sliding_window_.push_back(std::move(curr_points));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PointcloudLocalMap::GetNearlyNeighbor(sensor::LaserPoint const& point, uint16_t const& num, 
                                                                    double const& max_range, std::vector<sensor::LaserPoint>& res) const {
} 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
const std::vector<Eigen::Vector2f>& PointcloudLocalMap::ReadLocalMap() const {
    return localmap_;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointcloudLocalMap::localMapDownsampling(
        const std::deque<std::vector<Eigen::Vector2f>>& sliding_window) {
    localmap_.clear();
    for (const auto& frame : sliding_window) {
        localmap_.insert(localmap_.end(), frame.begin(), frame.end());  
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PointcloudLocalMap::createKDTreeFromLocalMap() {
    target_kdtree_database_.resize(2, localmap_.size());

    for(int i = 0; i < localmap_.size();i++) {
        target_kdtree_database_(0,i) = localmap_[i][0];    // x
        target_kdtree_database_(1,i) = localmap_[i][1];    // y 
    }

    target_kdtree_ = Nabo::NNSearchD::createKDTreeLinearHeap(target_kdtree_database_);
}

}
}
