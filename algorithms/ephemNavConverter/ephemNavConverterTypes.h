/*
 MIT License

 Copyright (c) 2025, University of Colorado at Boulder
 */

#ifndef FP32_FSW_XMERA_EPHEMNAVCONVERTERTYPES_H
#define FP32_FSW_XMERA_EPHEMNAVCONVERTERTYPES_H

#include <Eigen/Core>

#ifdef __cplusplus
extern "C" {
#endif

/*! Struct containing the ephemeris inputs needed by the algorithm. */
struct InputEphemerisData {
    double timeTag{};
    Eigen::Vector3d r_BdyZero_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_BdyZero_N = Eigen::Vector3d::Zero();
};

/*! Struct containing the translational navigation output produced by the algorithm. */
struct OutputNavTransData {
    double timeTag{};
    Eigen::Vector3d r_BN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_BN_N = Eigen::Vector3d::Zero();
};

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // FP32_FSW_XMERA_EPHEMNAVCONVERTERTYPES_H
