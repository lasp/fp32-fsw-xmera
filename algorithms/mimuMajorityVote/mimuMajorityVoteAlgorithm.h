#ifndef MIMU_MAJORITY_VOTE_ALGORITHM
#define MIMU_MAJORITY_VOTE_ALGORITHM

#include <Eigen/Core>
#include <array>

#include "mimuMajorityVoteTypes.h"

/*!@brief Module to compute the majority vote of the mimus. */
class MimuMajorityVoteAlgorithm {
   public:
    MimuMajorityVoteOutput update(const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B);
    void reset();                                                     //!< Reset fault persistence counters to zero
    void setOmegaThreshold(float omegaThresholdIn);                   //!< Setter method for omegaThreshold
    float getOmegaThreshold() const;                                  //!< Getter method for omegaThreshold
    void setFaultPersistenceLimit(uint32_t faultPersistenceLimitIn);  //!< Setter method for faultPersistenceLimit
    uint32_t getFaultPersistenceLimit() const;                        //!< Getter method for faultPersistenceLimit

   private:
    float omegaThreshold = 1.0F;          //!< Threshold to determine if a MIMU is faulted (rad/s)
    uint32_t faultPersistenceLimit = 1U;  //!< Number of consecutive faults needed to trigger faultDetected

    std::array<uint32_t, kMimuCount> faultPersistenceCount{};
};

#endif
