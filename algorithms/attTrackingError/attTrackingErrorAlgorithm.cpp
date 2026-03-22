#include "attTrackingErrorAlgorithm.h"
#include "attTrackingErrorTypes.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"

/*! This method computes the attitude and rate tracking errors between the spacecraft navigation attitude and the
 reference attitude and outputs the corresponding guidance errors.
 @param navIn navigation attitude inputs (sigma_BN and omega_BN_B)
 @param refIn reference attitude inputs (sigma_RN, omega_RN_N and domega_RN_N)
 @return AttGuidOutput guidance outputs (sigma_BR, omega_BR_B, omega_RN_B and domega_RN_B)
*/
AttGuidOutput AttTrackingErrorAlgorithm::update(const AttNavInput& navIn, const AttRefInput& refIn) {
    AttGuidOutput output{};

    // Compute attitude tracking error sigma_BR
    output.sigma_BR = subMrp(navIn.sigma_BN, refIn.sigma_RN);

    // Transform reference angular velocity from inertial to body frame components omega_RN_B
    const Eigen::Matrix3f dcm_BN = mrpToDcm(navIn.sigma_BN);
    output.omega_RN_B = dcm_BN * refIn.omega_RN_N;

    // Compute angular velocity tracking error omega_BR_B
    output.omega_BR_B = navIn.omega_BN_B - output.omega_RN_B;

    // Transform reference angular acceleration from inertial to body frame components domega_RN_B
    output.domega_RN_B = dcm_BN * refIn.domega_RN_N;

    return output;
}