#ifndef AVERAGE_MIMU_DATA_ALGORITHM_H
#define AVERAGE_MIMU_DATA_ALGORITHM_H

#include "averageMimuDataTypes.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>

#include <cstdint>

class AverageMimuDataAlgorithm {
   public:
    void setAveragingWindow(float window);                  //!< [s] Setter method for windowSec
    float getAveragingWindow() const;                       //!< [s] Getter method for windowSec
    void setDcmPltfToBdy(Eigen::Matrix3f const& dcm_BPIn);  //!< Setter method for dcm from platform to body
    Eigen::Matrix3f getDcmPltfToBdy() const;                //!< Getter method for dcm from platform to body
    OutputAverageAccelAngleVel update(InputPktsData const& localPkts) const;

   private:
    // Stored as nanoseconds so the per-sample comparison in update() is a pure
    // uint64_t compare. Float is only used at the public seconds-based API.
    std::uint64_t averagingWindowNs{0U};                   //!< [ns] Allowable time difference from "latest"
    Eigen::Matrix3f dcm_BP = Eigen::Matrix3f::Identity();  //!< [-] Transformation from the platform frame to body
};

#endif
