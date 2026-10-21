#include "dvGuidance.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>

/*! Builds the validated configuration and constructs the algorithm. */
void DvGuidance::reset(const uint64_t callTime) {
    auto config = DvGuidanceConfig::create(this->cmdDv_N, this->rHatPref_N);
    this->algorithm = std::make_unique<DvGuidanceAlgorithm>(config);
}

DvGuidanceConfig DvGuidance::toConfig() const { return DvGuidanceConfig::create(this->cmdDv_N, this->rHatPref_N); }

void DvGuidance::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvGuidance reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! Computes the burn-frame attitude reference from the configured delta-V command. */
void DvGuidance::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("DvGuidance reset() has not been called.");
    }

    const Eigen::Vector3f sigma_RN = this->algorithm->update();

    // The burn frame is inertially fixed: the zero-initialized reference rate and acceleration are left at zero.
    AttRefMsgF32Payload attRefOut{};
    eigenVectorToCArray(sigma_RN, attRefOut.sigma_RN);

    this->attRefOutMsg.write(attRefOut, this->moduleID, callTime);
}
