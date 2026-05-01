#include "mrpFeedback.h"

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

    this->rebuildAlgorithmConfig();
    this->algorithm.reset(sc, rwConfigParams, rwParamsIsLinked);
}

void MrpFeedback::updateState(const uint64_t callTime) {
    AttGuidMsgF32Payload guidCmd = this->guidInMsg();
    RWSpeedMsgF32Payload wheelSpeeds{};
    RWAvailabilityMsgPayload wheelsAvailability{};

    if (this->numRW > 0U) {
        wheelSpeeds = this->rwSpeedsInMsg();
        if (this->rwAvailInMsg.isLinked()) {
            wheelsAvailability = this->rwAvailInMsg();
        }
    }

    auto [controlOut, intFeedbackOut] = this->algorithm.update(callTime, guidCmd, wheelSpeeds, wheelsAvailability);

    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);
    this->intFeedbackTorqueOutMsg.write(&intFeedbackOut, this->moduleID, callTime);
}

void MrpFeedback::setK(const float gain) {
    if (!MrpFeedbackConfig::isValidK(gain)) {
        FSW_THROW_INVALID_ARGUMENT("Feedback gain K must not be negative");
    }
    this->K = gain;
    this->rebuildAlgorithmConfig();
}
float MrpFeedback::getK() const { return this->K; }

void MrpFeedback::setP(const float gain) {
    if (!MrpFeedbackConfig::isValidP(gain)) {
        FSW_THROW_INVALID_ARGUMENT("Feedback gain P must not be negative");
    }
    this->P = gain;
    this->rebuildAlgorithmConfig();
}
float MrpFeedback::getP() const { return this->P; }

void MrpFeedback::setKi(const float gain) {
    if (!MrpFeedbackConfig::isValidKi(gain)) {
        FSW_THROW_INVALID_ARGUMENT("Integral feedback gain Ki must not be negative");
    }
    this->Ki = gain;
    this->rebuildAlgorithmConfig();
}
float MrpFeedback::getKi() const { return this->Ki; }

void MrpFeedback::setIntegralLimit(const float limit) {
    if (!MrpFeedbackConfig::isValidIntegralLimit(limit)) {
        FSW_THROW_INVALID_ARGUMENT("Integral limit must not be negative");
    }
    this->integralLimit = limit;
    this->rebuildAlgorithmConfig();
}
float MrpFeedback::getIntegralLimit() const { return this->integralLimit; }

void MrpFeedback::setControlLawType(const int type) {
    this->controlLawType = static_cast<ControlLawType>(type);
    this->rebuildAlgorithmConfig();
}
int MrpFeedback::getControlLawType() const { return static_cast<int>(this->controlLawType); }

void MrpFeedback::setKnownTorquePntB_B(const Eigen::Vector3f& torque) {
    this->knownTorquePntB_B = torque;
    this->rebuildAlgorithmConfig();
}
Eigen::Vector3f MrpFeedback::getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }

void MrpFeedback::rebuildAlgorithmConfig() {
    const MrpFeedbackConfig cfg = MrpFeedbackConfig::create(
        this->K, this->P, this->Ki, this->integralLimit, this->controlLawType, this->knownTorquePntB_B);
    this->algorithm.setConfig(cfg);
}
