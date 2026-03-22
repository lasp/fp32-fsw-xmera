#include "attTrackingError.h"

#include "../../architecture/utilities/messageConversionHelpers.hpp"
#include "architecture/utilities/eigenSupport.h"
#include "attTrackingErrorTypes.h"

/*! This method performs a complete reset of the module. Local module variables that retain time varying states between
 function calls are reset to their default values.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingError::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("attTrackingError.attRefInMsg wasn't connected.");
    }
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("attTrackingError.attNavInMsg wasn't connected.");
    }
}

/*! The Update method performs reads the Navigation message (containing the spacecraft attitude information), and the
 Reference message (containing the desired attitude). It computes the attitude error and writes it in the Guidance
 message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingError::updateState(uint64_t callTime) {
    // Convert incoming double messages to floats
    AttRefMsgF32Payload refF32{};
    convert(this->attRefInMsg(), refF32);

    NavAttMsgF32Payload navF32{};
    convert(this->attNavInMsg(), navF32);

    // Convert input C arrays to eigen vectors
    const Eigen::Vector3f sigma_BN = cArrayToEigenVector(navF32.sigma_BN);
    const Eigen::Vector3f omega_BN_B = cArrayToEigenVector(navF32.omega_BN_B);

    const Eigen::Vector3f sigma_RN = cArrayToEigenVector(refF32.sigma_RN);
    const Eigen::Vector3f omega_RN_N = cArrayToEigenVector(refF32.omega_RN_N);
    const Eigen::Vector3f domega_RN_N = cArrayToEigenVector(refF32.domega_RN_N);

    // Create algorithm input structs
    AttNavInput navIn{};
    navIn.sigma_BN = sigma_BN;
    navIn.omega_BN_B = omega_BN_B;

    AttRefInput refIn{};
    refIn.sigma_RN = sigma_RN;
    refIn.omega_RN_N = omega_RN_N;
    refIn.domega_RN_N = domega_RN_N;

    // Call the attitude tracking error algorithm
    const AttGuidOutput guidOut = this->algorithm.update(navIn, refIn);

    // Create local container for output message
    AttGuidMsgF32Payload attGuidOutF32{};

    // Convert algorithm outputs from eigen vectors to C arrays
    eigenVectorToCArray(guidOut.sigma_BR, attGuidOutF32.sigma_BR);
    eigenVectorToCArray(guidOut.omega_BR_B, attGuidOutF32.omega_BR_B);
    eigenVectorToCArray(guidOut.omega_RN_B, attGuidOutF32.omega_RN_B);
    eigenVectorToCArray(guidOut.domega_RN_B, attGuidOutF32.domega_RN_B);

    // Convert floats to doubles
    AttGuidMsgPayload attGuidOut{};
    convert(attGuidOutF32, attGuidOut);

    // Write the attitude guidance output message
    this->attGuidOutMsg.write(&attGuidOut, this->moduleID, callTime);
}
