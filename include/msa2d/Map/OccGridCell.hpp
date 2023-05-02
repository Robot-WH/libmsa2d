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

#include <string>
#include "OccGridCellOperate.hpp"

namespace msa2d {
namespace map {

class OccGridCell {
public:
    /**
     * @brief Construct a new Occ Grid Cell object
     * 
     * @param type 对数概率法 probability, 计数法 cnt, TSDF
     */
    OccGridCell(std::string type = "probability") {
        if (type == "probability") {
            cell_operate_ = new OccGridCellOperationLogOdd; 
        } else if (type == "cnt") {
        } else if (type == "tsdf") {
        }
    }

    ~OccGridCell() {
        delete cell_operate_;  
    }

    /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
    bool isOccupied() const {
        return cell_operate_->isOccupied();   
    }

    bool isFree() const {
        return cell_operate_->isFree(); 
    }

    /**
   * Reset Cell to prior probability.
   */
    void resetGridCell() {
        cell_operate_->resetGridCell(); 
    }

        /**
   * Update cell as occupied
   * @param cell The cell.
   */
    void updateSetOccupied() {
        cell_operate_->updateSetOccupied(); 
    }

    /**
   * Update cell as free
   * @param cell The cell.
   */
    void updateSetFree() {
        cell_operate_->updateSetFree();  
    }

    void updateUnsetFree() {
        cell_operate_->updateUnsetFree(); 
    }

    float getGridProbability() const {
        return cell_operate_->getGridProbability(); 
    }

private:
    OccGridCellOperationBase* cell_operate_; 
};
}
}
