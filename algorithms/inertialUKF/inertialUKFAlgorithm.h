/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_INERTIAL_UKF_ALGORITHM_H
#define F32XIMERA_INERTIAL_UKF_ALGORITHM_H

#include <Eigen/Core>

/*! @brief Abstracted star tracker measurement input. */
struct STAttInput {
    float timeTag{};
    Eigen::Vector3f MRP_BdyInrtl = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
};

/*! @brief Abstracted gyro measurement input. */
struct GyroInput {
    Eigen::Vector3f gyro_B = Eigen::Vector3f::Zero();
};

/*! @brief Abstracted RW speeds input. */
struct RWSpeedsInput {
    Eigen::Vector4f wheelSpeeds = Eigen::Vector4f::Zero();
};

/*! @brief Abstracted RW array configuration input. */
struct RWArrayConfigInput {
    int numRW{};
};

/*! @brief Abstracted vehicle configuration input. */
struct VehicleConfigInput {
    Eigen::Matrix3f ISCPntB_B = Eigen::Matrix3f::Identity();
    float massSC{};
};

/*! @brief Abstracted navigation attitude output. */
struct NavAttOutput {
    double timeTag{};
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f vehSunPntBdy = Eigen::Vector3f::Zero();
};

/*! @brief Abstracted inertial filter output. */
struct InertialFilterOutput {
    double timeTag{};
    int numObs{};
};

/*! @brief Combined output of the inertial UKF algorithm. */
struct InertialUKFOutput {
    NavAttOutput navAtt{};
    InertialFilterOutput filter{};
};

/*! @brief Inertial UKF pass-through algorithm class. */
class InertialUKFAlgorithm {
   public:
    static InertialUKFOutput updateState(const STAttInput& stAtt,
                                         const GyroInput& gyro,
                                         const RWSpeedsInput& rwSpeeds,
                                         const RWArrayConfigInput& rwConfig,
                                         const VehicleConfigInput& vehConfig);
};

#endif
