/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_MIMU_MAJORITY_VOTE_TYPES_H
#define F32XIMERA_MIMU_MAJORITY_VOTE_TYPES_H

constexpr uint32_t MAX_IMU_VEH_COUNT = 4U;

/*! @brief Input angular velocity from a single IMU */
struct MimuInput {
    Eigen::Vector3f angVelBody{}; /*!< [rad/s] Angular velocity in body frame */
};

/*! @brief Output from the MIMU majority vote algorithm */
struct MimuMajorityVoteOutput {
    Eigen::Vector3f avgAngVelBody{}; /*!< [rad/s] Averaged angular velocity in body frame */
    bool faultDetected{};            /*!< Whether a MIMU fault was detected */
    int mimuIndexFaulted{};          /*!< Index of faulted MIMU (-1 if no fault) */
};

#endif  // F32XIMERA_MIMU_MAJORITY_VOTE_TYPES_H
