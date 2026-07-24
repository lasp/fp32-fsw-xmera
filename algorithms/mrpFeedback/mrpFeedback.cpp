#include "mrpFeedback.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

void MrpFeedback::reset(const uint64_t callTime) {
    if (this->rwParamsInMsg.isLinked() && !this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
    }
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("MrpFeedback.vehConfigInMsg wasn't connected.");
    }

    const Eigen::Matrix3f inertia = cArrayToEigenMatrix3(this->vehConfigInMsg().ISCPntB_B);
    RWArrayConfigMsgF32Payload rwConfigParams{};
    bool rwParamsIsLinked{};
    if (this->rwParamsInMsg.isLinked()) {
        rwConfigParams = this->rwParamsInMsg();
        rwParamsIsLinked = true;
    }
    this->numRW = static_cast<uint32_t>(rwConfigParams.numRW);

    const MrpFeedbackControlParameters controlParameters{
        .K = this->K,
        .P = this->P,
        .Ki = this->Ki,
        .integralLimit = this->integralLimit,
        .controlLawType = this->controlLawType,
        .controlPeriod = this->controlPeriod,
    };
    auto config = MrpFeedbackConfig::create(controlParameters, this->knownTorquePntB_B, inertia);
    this->algorithm = std::make_unique<MrpFeedbackAlgorithm>(config);
    this->algorithm->reset(rwConfigParams, rwParamsIsLinked);
}

void MrpFeedback::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpFeedback reset() has not been called.");
    }

    AttGuidMsgF32Payload guidCmd = this->guidInMsg();
    RWSpeedMsgF32Payload wheelSpeeds{};
    RWAvailabilityMsgPayload wheelsAvailability{};

    if (this->numRW > 0U) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    auto [controlOut, intFeedbackOut] = this->algorithm->update(guidCmd, wheelSpeeds, wheelsAvailability);

    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);
    this->intFeedbackTorqueOutMsg.write(&intFeedbackOut, this->moduleID, callTime);
}
