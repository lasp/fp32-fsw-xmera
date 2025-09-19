/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "sunlineEphemAlgorithm.h"

NavAttMsgF32Payload SunlineEphemAlgorithm::updateState(const EphemerisMsgF32Payload &sunPos,
                                                       const NavTransMsgF32Payload &scPos,
                                                       const NavAttMsgF32Payload &scAtt) const {
    // Get sun position
    const Eigen::Vector3f rSun(sunPos.r_BdyZero_N[0], sunPos.r_BdyZero_N[1], sunPos.r_BdyZero_N[2]);
    const Eigen::Vector3f rSc(scPos.r_BN_N[0], scPos.r_BN_N[1], scPos.r_BN_N[2]);
    // Difference in inertial frame
    const Eigen::Vector3f r_SB_N = rSun - rSc;
    Eigen::Vector3f r_SB_N_hat = Eigen::Vector3f::Zero();
    if (r_SB_N.norm() > std::numeric_limits<float>::epsilon()) {
        r_SB_N_hat = r_SB_N;
        r_SB_N_hat.normalize();  // in-place unit-length
    }
    // Build DCM from spacecraft attitude
    const Eigen::Vector3f sigma_BN(scAtt.sigma_BN[0], scAtt.sigma_BN[1], scAtt.sigma_BN[2]);
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);

    // Rotate into body frame
    Eigen::Vector3f r_SB_B_hat = dcm_BN * r_SB_N_hat;

    // Ensure unit length (or zero)
    if (r_SB_B_hat.norm() > std::numeric_limits<float>::epsilon()) {
        r_SB_B_hat.normalize();  // in-place unit-length
    } else {
        r_SB_B_hat.setZero();  // explicit zero
    }

    auto outputSunline = NavAttMsgF32Payload{}; /* [-] Output sunline estimate data */
    for (int i = 0; i < 3; i++) {
        outputSunline.vehSunPntBdy[i] = r_SB_B_hat[i];
    }
    return outputSunline;
}
