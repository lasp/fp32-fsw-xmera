#ifndef MIMU_MAJORITY_VOTE_ALGORITHM
#define MIMU_MAJORITY_VOTE_ALGORITHM

#include "mimuMajorityVoteTypes.h"
#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <Eigen/Core>
#include <array>
#include <cstdint>

/*! @brief Result of one majority vote over the kMimuCount IMU measurements of a single quantity. */
struct MimuVoteResult {
    Eigen::Vector3f average{}; /*!< Averaged measurement (outlier-excluded once a fault persists) */
    bool faultDetected{};      /*!< Whether an IMU was rejected for this quantity */
    std::array<float, kMimuCount> imuDifferenceMag{}; /*!< Each IMU's difference magnitude from the 3-IMU average */
    std::array<bool, kMimuCount> imuValid{};          /*!< Whether each IMU is considered valid for this quantity */
};

/*! @brief Output from the MIMU majority vote algorithm: independent gyro and accel votes. */
struct MimuMajorityVoteOutput {
    MimuVoteResult gyro;  /*!< [rad/s] Angular-velocity vote; gyro.average is in the body frame */
    MimuVoteResult accel; /*!< [m/s^2] Acceleration vote; accel.average is in the body frame */
};

/*!
 * @brief Validated configuration for the MIMU majority vote algorithm.
 *
 * An instance can only exist with finite, strictly positive gyro and accel thresholds and strictly
 * positive gyro and accel fault persistence limits. Construct via MimuMajorityVoteConfig::create(...).
 */
class MimuMajorityVoteConfig final {
   public:
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    static MimuMajorityVoteConfig create(float omegaThreshold,
                                         uint32_t gyroFaultPersistenceLimit,
                                         float accelThreshold,
                                         uint32_t accelFaultPersistenceLimit) {
        if (!isValidOmegaThreshold(omegaThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("mimuMajorityVote: omegaThreshold must be finite and > 0");
        }
        if (!isValidGyroFaultPersistenceLimit(gyroFaultPersistenceLimit)) {
            FSW_THROW_INVALID_ARGUMENT("mimuMajorityVote: gyroFaultPersistenceLimit must be > 0");
        }
        if (!isValidAccelThreshold(accelThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("mimuMajorityVote: accelThreshold must be finite and > 0");
        }
        if (!isValidAccelFaultPersistenceLimit(accelFaultPersistenceLimit)) {
            FSW_THROW_INVALID_ARGUMENT("mimuMajorityVote: accelFaultPersistenceLimit must be > 0");
        }
        return {omegaThreshold, gyroFaultPersistenceLimit, accelThreshold, accelFaultPersistenceLimit};
    }

    static bool isValidOmegaThreshold(float omegaThreshold) {
        return fsw::is_finite(omegaThreshold) && omegaThreshold > 0.0F;
    }
    static bool isValidGyroFaultPersistenceLimit(uint32_t gyroFaultPersistenceLimit) {
        return gyroFaultPersistenceLimit > 0U;
    }
    static bool isValidAccelThreshold(float accelThreshold) {
        return fsw::is_finite(accelThreshold) && accelThreshold > 0.0F;
    }
    static bool isValidAccelFaultPersistenceLimit(uint32_t accelFaultPersistenceLimit) {
        return accelFaultPersistenceLimit > 0U;
    }

    float getOmegaThreshold() const { return omegaThreshold; }
    uint32_t getGyroFaultPersistenceLimit() const { return gyroFaultPersistenceLimit; }
    float getAccelThreshold() const { return accelThreshold; }
    uint32_t getAccelFaultPersistenceLimit() const { return accelFaultPersistenceLimit; }

   private:
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    MimuMajorityVoteConfig(float omegaThreshold,
                           uint32_t gyroFaultPersistenceLimit,
                           float accelThreshold,
                           uint32_t accelFaultPersistenceLimit)
        : omegaThreshold(omegaThreshold),
          gyroFaultPersistenceLimit(gyroFaultPersistenceLimit),
          accelThreshold(accelThreshold),
          accelFaultPersistenceLimit(accelFaultPersistenceLimit) {}

    float omegaThreshold;
    uint32_t gyroFaultPersistenceLimit;
    float accelThreshold;
    uint32_t accelFaultPersistenceLimit;
};

/*!@brief Module to compute the majority vote of the mimus. */
class MimuMajorityVoteAlgorithm final {
   public:
    explicit MimuMajorityVoteAlgorithm(const MimuMajorityVoteConfig& config);
    void setConfig(const MimuMajorityVoteConfig& config);  //!< Install configuration parameters
    void reInitialize();                                   //!< Reset gyro and accel persistence counters to zero
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    MimuMajorityVoteOutput update(const std::array<Eigen::Vector3f, kMimuCount>& imuOmegas_BN_B,
                                  const std::array<Eigen::Vector3f, kMimuCount>& imuAccels_B);

   private:
    MimuMajorityVoteConfig cfg;
    std::array<uint32_t, kMimuCount> gyroFaultPersistenceCount{};
    std::array<uint32_t, kMimuCount> accelFaultPersistenceCount{};
};

#endif
