#ifndef F32XIMERA_MIMU_MAJORITY_VOTE_TYPES_H
#define F32XIMERA_MIMU_MAJORITY_VOTE_TYPES_H

#include <Eigen/Core>
#include <array>
#include <cstdint>

constexpr uint32_t MAX_IMU_VEH_COUNT = 4U;

/*! @brief Input angular velocity from a single IMU */
struct MimuInput {
    Eigen::Vector3f angVelBody{}; /*!< [rad/s] Angular velocity in body frame */
};

/*! @brief Output from the MIMU majority vote algorithm */
struct MimuMajorityVoteOutput {
    Eigen::Vector3f avgAngVelBody{}; /*!< [rad/s] Averaged angular velocity in body frame */
    bool faultDetected{};            /*!< Whether a MIMU fault was detected */
    std::array<float, MAX_IMU_VEH_COUNT>
        omegaDifferencesMag{}; /*!< [rad/s] Each IMU's difference magnitude from the 3-IMU average */
    std::array<bool, MAX_IMU_VEH_COUNT> validImus{}; /*!< Whether each IMU is considered valid */
};

#endif  // F32XIMERA_MIMU_MAJORITY_VOTE_TYPES_H
