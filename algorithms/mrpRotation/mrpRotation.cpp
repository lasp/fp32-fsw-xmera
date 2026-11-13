#include "mrpRotation.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <stdexcept>

/*! @brief Validate that the required input message is linked, build the algorithm's configuration
 from the adapter's stored properties, and (re)construct the embedded algorithm. Construction starts the
 integrated rotation at identity.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpRotation::reset(const uint64_t callTime) {
    // check if the required input message is connected
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("mrpRotation.attRefInMsg wasn't connected.");
    }

    auto config =
        MrpRotationConfig::create(this->sigma_RcR0, this->sigma_RpRc, this->omega_RRp_Rp, this->controlPeriod);
    this->algorithm = std::make_unique<MrpRotationAlgorithm>(config);
}

MrpRotationConfig MrpRotation::toConfig() const {
    return MrpRotationConfig::create(this->sigma_RcR0, this->sigma_RpRc, this->omega_RRp_Rp, this->controlPeriod);
}

void MrpRotation::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpRotation reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
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

    const MrpRotationAttRefInputs attRef{
        cArrayToEigenVector(inputRefPayload.sigma_RN),
        cArrayToEigenVector(inputRefPayload.omega_RN_N),
        cArrayToEigenVector(inputRefPayload.domega_RN_N),
    };

    const MrpRotationOutput out = this->algorithm->update(attRef);

    AttRefMsgF32Payload attRefOut{};
    eigenVectorToCArray(out.sigma_RN, attRefOut.sigma_RN);
    eigenVectorToCArray(out.omega_RN_N, attRefOut.omega_RN_N);
    eigenVectorToCArray(out.domega_RN_N, attRefOut.domega_RN_N);

    /*! - write attitude guidance reference output */
    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}

/*! @brief Restart the algorithm's integrated rotation at identity. */
void MrpRotation::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpRotation reset() has not been called.");
    }
    this->algorithm->reInitialize();
}
