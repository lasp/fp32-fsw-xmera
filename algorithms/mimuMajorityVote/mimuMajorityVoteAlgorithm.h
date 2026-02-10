#ifndef MIMU_MAJORITY_VOTE_ALGORITHM
#define MIMU_MAJORITY_VOTE_ALGORITHM

#include <Eigen/Core>
#include <array>
#include <cstdint>

#include "mimuMajorityVoteTypes.h"

/*!@brief Module to compute the majority vote of the mimus. */
class MimuMajorityVoteAlgorithm {
   public:
    MimuMajorityVoteOutput update(const std::array<MimuInput, MAX_IMU_VEH_COUNT> &imuInputs, size_t numberOfImus);
    void setOmegaThreshold(float omegaThresholdIn);  //!< Setter method for omegaThreshold
    float getOmegaThreshold() const;                 //!< Getter method for omegaThreshold

   private:
    float omegaThreshold = 1.0F;  // The threshold in which we will determine one of the mimus is faulted (rad/s)
    std::array<float, MAX_IMU_VEH_COUNT - 1U> omegaDifferencesMag = {};
};

#endif
