#include "inertial3D.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <cstdint>

/*! This method builds the validated configuration and constructs the algorithm.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3D::reset(const uint64_t callTime) {
    auto config = Inertial3DConfig::create(this->sigma_RN);
    this->algorithm = std::make_unique<Inertial3DAlgorithm>(config);
}

/*! This method creates a fixed attitude reference message. The desired orientation is
    defined by the module's sigma_RN configuration.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Inertial3D::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("Inertial3D reset() has not been called.");
    }

    const Eigen::Vector3f referenceMrp = this->algorithm->update();

    AttRefMsgF32Payload attRefOut{};
    eigenVectorToCArray(referenceMrp, attRefOut.sigma_RN);
    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}
