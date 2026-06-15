#include "attTrackingError.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

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
    auto config = AttTrackingErrorConfig::create();
    this->algorithm = std::make_unique<AttTrackingErrorAlgorithm>(config);
}

/*! The Update method performs reads the Navigation message (containing the spacecraft attitude information), and the
 Reference message (containing the desired attitude). It computes the attitude error and writes it in the Guidance
 message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void AttTrackingError::updateState(uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("AttTrackingError reset() has not been called.");
    }

    NavAttMsgF32Payload navF32 = this->attNavInMsg();
    AttRefMsgF32Payload refF32 = this->attRefInMsg();

    AttNavInput navIn{};
    navIn.sigma_BN = cArrayToEigenVector(navF32.sigma_BN);
    navIn.omega_BN_B = cArrayToEigenVector(navF32.omega_BN_B);

    AttRefInput refIn{};
    refIn.sigma_RN = cArrayToEigenVector(refF32.sigma_RN);
    refIn.omega_RN_N = cArrayToEigenVector(refF32.omega_RN_N);
    refIn.domega_RN_N = cArrayToEigenVector(refF32.domega_RN_N);

    const AttGuidOutput guidOut = this->algorithm->update(navIn, refIn);

    AttGuidMsgF32Payload attGuidOutF32{};
    eigenVectorToCArray(guidOut.sigma_BR, attGuidOutF32.sigma_BR);
    eigenVectorToCArray(guidOut.omega_BR_B, attGuidOutF32.omega_BR_B);
    eigenVectorToCArray(guidOut.omega_RN_B, attGuidOutF32.omega_RN_B);
    eigenVectorToCArray(guidOut.domega_RN_B, attGuidOutF32.domega_RN_B);

    this->attGuidOutMsg.write(attGuidOutF32, this->moduleID, callTime);
}
