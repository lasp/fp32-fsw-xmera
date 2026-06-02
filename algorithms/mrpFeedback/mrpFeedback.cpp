#include "mrpFeedback.h"

#include "architecture/utilities/eigenSupport.h"
#include "utilities/xmeraLifecycleException.h"
#include <algorithm>
#include <array>
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
    const Eigen::Matrix3f ISCPntB_B = cArrayToEigenMatrix3(sc.ISCPntB_B);

    RWArrayConfigMsgF32Payload rwConfigParams{};
    if (this->rwParamsInMsg.isLinked()) {
        rwConfigParams = this->rwParamsInMsg();
    }
    this->numRW = static_cast<uint32_t>(rwConfigParams.numRW);

    const Eigen::Matrix<float, 3, RW_EFF_CNT> Gs_B =
        cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(rwConfigParams.GsMatrix_B);
    std::array<float, RW_EFF_CNT> JsList{};
    std::copy(std::begin(rwConfigParams.JsList), std::end(rwConfigParams.JsList), JsList.begin());

    auto config = MrpFeedbackConfig::create(this->K,
                                            this->P,
                                            this->Ki,
                                            this->integralLimit,
                                            this->controlLawType,
                                            this->knownTorquePntB_B,
                                            ISCPntB_B,
                                            rwConfigParams.numRW,
                                            Gs_B,
                                            JsList);
    this->algorithm = std::make_unique<MrpFeedbackAlgorithm>(config);
    this->algorithm->reset();
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
