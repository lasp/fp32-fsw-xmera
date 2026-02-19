#ifndef MIMU_MAJORITY_VOTE_ALGORITHM
#define MIMU_MAJORITY_VOTE_ALGORITHM

#include <Eigen/Core>
#include <array>
#include <cstdint>

#include "mimuMajorityVoteTypes.h"

/*!@brief Module to compute the majority vote of the mimus. */
class MimuMajorityVoteAlgorithm {
   public:
    MimuMajorityVoteOutput update(const std::array<MimuInput, MAX_IMU_VEH_COUNT>& imuInputs);
    void setOmegaThreshold(float omegaThresholdIn);  //!< Setter method for omegaThreshold
    float getOmegaThreshold() const;                 //!< Getter method for omegaThreshold
    void setNumberOfImus(size_t numberOfImusIn);     //!< Setter method for numberOfImus
    size_t getNumberOfImus() const;                  //!< Getter method for numberOfImus

   private:
    float omegaThreshold = 1.0F;  // The threshold in which we will determine one of the mimus is faulted (rad/s)
    size_t numberOfImus = 0U;     // Fixed number of IMUs used in each update call
};

#endif
