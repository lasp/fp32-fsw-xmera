#include "attTrackingError.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/messageConversionHelpers.hpp"

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
    // Read F32 input messages
    AttRefMsgF32Payload refF32 = this->attRefInMsg();
    NavAttMsgF32Payload navF32 = this->attNavInMsg();

    // Convert input C arrays to eigen vectors
    const Eigen::Vector3f sigma_BN = Eigen::Map<const Eigen::Vector3f>(navF32.sigma_BN);
    const Eigen::Vector3f omega_BN_B = Eigen::Map<const Eigen::Vector3f>(navF32.omega_BN_B);
    const Eigen::Vector3f sigma_RN = Eigen::Map<const Eigen::Vector3f>(refF32.sigma_RN);
    const Eigen::Vector3f omega_RN_N = Eigen::Map<const Eigen::Vector3f>(refF32.omega_RN_N);
    const Eigen::Vector3f domega_RN_N = Eigen::Map<const Eigen::Vector3f>(refF32.domega_RN_N);

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

    // Write the attitude guidance float output message
    this->attGuidOutMsg.write(attGuidOutF32, this->moduleID, callTime);
}
