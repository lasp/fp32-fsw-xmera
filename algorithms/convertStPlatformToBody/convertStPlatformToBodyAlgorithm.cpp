/*
 MIT License

 Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "convertStPlatformToBodyAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

StAttitudeOutput ConvertStPlatformToBodyAlgorithm::update(StSensorInput& stSensorIn) const {
    // Convert the star tracker inertial attitude from quaternion to MRP
    const Eigen::Vector4f ep_CN = cArrayToEigenVector(stSensorIn.qInrtl2Case);
    const Eigen::Vector3f sigma_CN = epToMrp(ep_CN);

    // Compute hub inertial attitude using specified dcm_CB
    const Eigen::Vector3f sigma_BC = dcmToMrp<float>(this->dcm_CB.transpose());
    const Eigen::Vector3f sigma_BN = addMrp(sigma_CN, sigma_BC);

    // Compute hub inertial angular velocity using specified dcm_CB
    const Eigen::Vector3f omega_CN_C = cArrayToEigenVector3<float>(stSensorIn.omega_CN_C);
    const Eigen::Vector3f omega_BN_B = this->dcm_CB.transpose() * omega_CN_C;

    // Build output
    StAttitudeOutput stAttOut{};
    stAttOut.timeTag = stSensorIn.timeTag;
    eigenVectorToCArray(sigma_BN, stAttOut.sigma_BN);
    eigenVectorToCArray(omega_BN_B, stAttOut.omega_BN_B);
    eigenMatrixToCArray(this->dcm_CB, stAttOut.dcm_CB);
    return stAttOut;
}

void ConvertStPlatformToBodyAlgorithm::setDcmCB(const Eigen::Matrix3f& dcm_CB) { this->dcm_CB = dcm_CB; }

const Eigen::Matrix3f& ConvertStPlatformToBodyAlgorithm::getDcmCB() const { return this->dcm_CB; }
