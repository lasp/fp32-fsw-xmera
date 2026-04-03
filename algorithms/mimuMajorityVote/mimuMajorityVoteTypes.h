#ifndef F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H
#define F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H

#include "msgPayloadDef/definitions.h"

#include <Eigen/Core>
#include <array>

/*! @brief Output from the MIMU majority vote algorithm */
struct MimuMajorityVoteOutput {
    Eigen::Vector3f avgOmega_BN_B{}; /*!< [rad/s] Averaged angular velocity in body frame */
    bool faultDetected{};            /*!< Whether a MIMU fault was detected */
    std::array<float, kMimuCount>
        omegaDifferencesMag{};                /*!< [rad/s] Each IMU's difference magnitude from the 3-IMU average */
    std::array<bool, kMimuCount> validImus{}; /*!< Whether each IMU is considered valid */
};

#endif  // F32XMERA_MIMU_MAJORITY_VOTE_TYPES_H
