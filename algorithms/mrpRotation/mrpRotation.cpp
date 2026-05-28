#include "mrpRotation.h"
#include "utilities/xmeraLifecycleException.h"
#include <architecture/utilities/eigenSupport.h>
#include <stdexcept>

/*! @brief Validate that the required input message is linked, rebuild the algorithm's
 configuration from the adapter's stored properties (which captures whether the optional
 desiredAttInMsg is connected), and reset the embedded algorithm.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpRotation::reset(const uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("mrpRotation.attRefInMsg wasn't connected.");
    }

    auto config = MrpRotationConfig::create(this->sigma_RR0, this->omega_RR0_R, this->desiredAttInMsg.isLinked());
    this->algorithm = std::make_unique<MrpRotationAlgorithm>(config);
    this->algorithm->reset();
}

/*! @brief Take the input attitude reference frame and superimpose the algorithm's MRP rotation on
 top of it, producing the output guidance reference message.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpRotation::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpRotation reset() has not been called.");
    }

    const AttRefMsgF32Payload inputRefPayload = this->attRefInMsg();
    AttStateMsgF32Payload attStatePayload{};
    if (this->desiredAttInMsg.isLinked()) {
        attStatePayload = this->desiredAttInMsg();
    }

    const MrpRotationAttRefInputs attRef{
        cArrayToEigenVector(inputRefPayload.sigma_RN),
        cArrayToEigenVector(inputRefPayload.omega_RN_N),
        cArrayToEigenVector(inputRefPayload.domega_RN_N),
    };
    const MrpRotationAttStateInputs attState{
        cArrayToEigenVector(attStatePayload.state),
        cArrayToEigenVector(attStatePayload.rate),
    };

    const MrpRotationOutput out = this->algorithm->update(callTime, attRef, attState);

    AttRefMsgF32Payload attRefOut{};
    eigenVectorToCArray(out.sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, attRefOut.domega_RN_N);

    /*! - write attitude guidance reference output */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}
