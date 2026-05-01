#include "mrpFeedback.h"

#include "utilities/xmeraLifecycleException.h"
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

    const VehicleConfigMsgF32Payload sc = this->vehConfigInMsg();
    RWArrayConfigMsgF32Payload rwConfigParams{};
    bool rwParamsIsLinked{};
    if (this->rwParamsInMsg.isLinked()) {
        rwConfigParams = this->rwParamsInMsg();
        rwParamsIsLinked = true;
    }
    this->numRW = static_cast<uint32_t>(rwConfigParams.numRW);

    auto config = MrpFeedbackConfig::create(
        this->K, this->P, this->Ki, this->integralLimit, this->controlLawType, this->knownTorquePntB_B);
    this->algorithm = std::make_unique<MrpFeedbackAlgorithm>(config);
    this->algorithm->reset(sc, rwConfigParams, rwParamsIsLinked);
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

    auto [controlOut, intFeedbackOut] = this->algorithm->update(callTime, guidCmd, wheelSpeeds, wheelsAvailability);

    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);
    this->intFeedbackTorqueOutMsg.write(&intFeedbackOut, this->moduleID, callTime);
}
