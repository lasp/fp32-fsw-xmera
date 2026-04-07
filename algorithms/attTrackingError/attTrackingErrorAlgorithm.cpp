#include "attTrackingErrorAlgorithm.h"

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

/*! This method computes attitude and rate tracking errors between the navigation attitude and the reference attitude,
 and outputs the corresponding guidance errors.
 @return Attitude guidance message
 @param attRefInMsg Input msg of reference attitude
 @param attNavInMsg Input msg measured attitude
 */
AttGuidMsgF32Payload AttTrackingErrorAlgorithm::update(AttRefMsgF32Payload& attRefInMsg,
                                                       NavAttMsgF32Payload& attNavInMsg) const {
    // Extract MRPs of reference and body frame, both defined relative to inertial frame
    const Eigen::Vector3f sigma_RN = cArrayToEigenVector(attRefInMsg.sigma_RN);
    const Eigen::Vector3f sigma_BN = cArrayToEigenVector(attNavInMsg.sigma_BN);

    // Compute attitude tracking error sigma_BR
    const Eigen::Vector3f sigma_BR = subMrp(sigma_BN, sigma_RN);

    // Transform reference angular velocity from inertial to body frame components omega_RN_B
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f omega_RN_N = cArrayToEigenVector(attRefInMsg.omega_RN_N);
    const Eigen::Vector3f omega_RN_B = dcm_BN * omega_RN_N;

    // Compute angular velocity tracking error omega_BR_B
    const Eigen::Vector3f omega_BN_B = cArrayToEigenVector(attNavInMsg.omega_BN_B);
    const Eigen::Vector3f omega_BR_B = omega_BN_B - omega_RN_B;

    // Transform reference angular acceleration from inertial to body frame components domega_RN_B
    const Eigen::Vector3f domega_RN_N = cArrayToEigenVector(attRefInMsg.domega_RN_N);
    const Eigen::Vector3f domega_RN_B = dcm_BN * domega_RN_N;

    // Populate attitude guidance output message
    AttGuidMsgF32Payload attGuidOut{};
    eigenVectorToCArray(omega_RN_B, attGuidOut.omega_RN_B);
    eigenVectorToCArray(omega_BR_B, attGuidOut.omega_BR_B);
    eigenVectorToCArray(sigma_BR, attGuidOut.sigma_BR);
    eigenVectorToCArray(domega_RN_B, attGuidOut.domega_RN_B);

    return attGuidOut;
}
