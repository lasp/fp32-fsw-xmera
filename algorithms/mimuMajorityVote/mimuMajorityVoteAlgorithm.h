#ifndef MIMU_MAJORITY_VOTE_ALGORITHM
#define MIMU_MAJORITY_VOTE_ALGORITHM

#include "mimuMajorityVoteTypes.h"
#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <Eigen/Core>
#include <array>
#include <cstdint>

/*! @brief Output from the MIMU majority vote algorithm */
struct MimuMajorityVoteOutput {
    Eigen::Vector3f avgOmega_BN_B{}; /*!< [rad/s] Averaged angular velocity in body frame */
    bool faultDetected{};            /*!< Whether a MIMU fault was detected */
    std::array<float, kMimuCount>
        omegaDifferencesMag{};                /*!< [rad/s] Each IMU's difference magnitude from the 3-IMU average */
    std::array<bool, kMimuCount> validImus{}; /*!< Whether each IMU is considered valid */
};

/*!
 * @brief Validated configuration for the MIMU majority vote algorithm.
 *
 * An instance can only exist with a finite, strictly positive omega threshold and a strictly
 * positive fault persistence limit. Construct via MimuMajorityVoteConfig::create(...).
 */
class MimuMajorityVoteConfig final {
   public:
    static MimuMajorityVoteConfig create(float omegaThreshold, uint32_t faultPersistenceLimit) {
        if (!isValidOmegaThreshold(omegaThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("mimuMajorityVote: omegaThreshold must be finite and > 0");
        }
        if (!isValidFaultPersistenceLimit(faultPersistenceLimit)) {
            FSW_THROW_INVALID_ARGUMENT("mimuMajorityVote: faultPersistenceLimit must be > 0");
        }
        return {omegaThreshold, faultPersistenceLimit};
    }

    static bool isValidOmegaThreshold(float omegaThreshold) {
        return fsw::is_finite(omegaThreshold) && omegaThreshold > 0.0F;
    }
    static bool isValidFaultPersistenceLimit(uint32_t faultPersistenceLimit) { return faultPersistenceLimit > 0U; }

    float getOmegaThreshold() const { return omegaThreshold; }
    uint32_t getFaultPersistenceLimit() const { return faultPersistenceLimit; }

   private:
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    MimuMajorityVoteConfig(float omegaThreshold, uint32_t faultPersistenceLimit)
        : omegaThreshold(omegaThreshold), faultPersistenceLimit(faultPersistenceLimit) {}

    float omegaThreshold;
    uint32_t faultPersistenceLimit;
};

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
