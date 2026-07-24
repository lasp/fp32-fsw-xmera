#include "mrpFeedback.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <algorithm>
#include <array>
#include <optional>
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

    std::optional<MrpFeedbackInputRwData> rwConfiguration;
    this->numRW = 0U;
    if (this->rwParamsInMsg.isLinked()) {
        const RWArrayConfigMsgF32Payload rwConfigParams = this->rwParamsInMsg();
        MrpFeedbackInputRwData rwData{};
        rwData.GsMatrix_B = cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(rwConfigParams.GsMatrix_B);
        std::copy(std::begin(rwConfigParams.JsList), std::end(rwConfigParams.JsList), std::begin(rwData.JsList));
        rwData.numRW = static_cast<uint32_t>(rwConfigParams.numRW);
        if (this->rwAvailInMsg.isLinked()) {
            const RWAvailabilityMsgPayload availabilityMsg = this->rwAvailInMsg();
            std::copy(std::begin(availabilityMsg.wheelAvailability),
                      std::end(availabilityMsg.wheelAvailability),
                      std::begin(rwData.wheelAvailability));
        }
        this->numRW = rwData.numRW;
        rwConfiguration = rwData;
    }

    const MrpFeedbackControlParameters controlParameters{
        .K = this->K,
        .P = this->P,
        .Ki = this->Ki,
        .integralLimit = this->integralLimit,
        .controlLawType = this->controlLawType,
        .controlPeriod = this->controlPeriod,
    };
    auto config = MrpFeedbackConfig::create(controlParameters, this->knownTorquePntB_B, inertia, rwConfiguration);
    this->algorithm = std::make_unique<MrpFeedbackAlgorithm>(config);
    this->algorithm->reset();
}

void MrpFeedback::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpFeedback reset() has not been called.");
    }

    const AttGuidMsgF32Payload guidCmd = this->guidInMsg();
    const MrpFeedbackInputGuidance attGuidInput{
        cArrayToEigenVector(guidCmd.sigma_BR),
        cArrayToEigenVector(guidCmd.omega_BR_B),
        cArrayToEigenVector(guidCmd.omega_RN_B),
        cArrayToEigenVector(guidCmd.domega_RN_B),
    };

    std::array<float, RW_EFF_CNT> wheelSpeeds{};
    if (this->numRW > 0U) {
        const RWSpeedMsgF32Payload wheelSpeedsMsg = this->rwSpeedsInMsg();
        std::copy(
            std::begin(wheelSpeedsMsg.wheelSpeeds), std::end(wheelSpeedsMsg.wheelSpeeds), std::begin(wheelSpeeds));
    }

    const MrpFeedbackOutput out = this->algorithm->update(attGuidInput, wheelSpeeds);

    CmdTorqueBodyMsgF32Payload controlOut{};
    eigenVectorToCArray(out.controlTorque, controlOut.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&controlOut, moduleID, callTime);

    CmdTorqueBodyMsgF32Payload intFeedbackOut{};
    eigenVectorToCArray(out.integralFeedbackTorque, intFeedbackOut.torqueRequestBody);
    this->intFeedbackTorqueOutMsg.write(&intFeedbackOut, this->moduleID, callTime);
}
