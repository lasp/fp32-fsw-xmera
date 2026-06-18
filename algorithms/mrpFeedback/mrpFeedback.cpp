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

    const AttGuidMsgF32Payload guidCmd = this->guidInMsg();
    MrpFeedbackGuidInput guid;
    guid.sigma_BR = cArrayToEigenVector(guidCmd.sigma_BR);
    guid.omega_BR_B = cArrayToEigenVector(guidCmd.omega_BR_B);
    guid.omega_RN_B = cArrayToEigenVector(guidCmd.omega_RN_B);
    guid.domega_RN_B = cArrayToEigenVector(guidCmd.domega_RN_B);

    Eigen::Vector<float, RW_EFF_CNT> wheelSpeeds = Eigen::Vector<float, RW_EFF_CNT>::Zero();
    std::array<bool, RW_EFF_CNT> wheelAvailability{};
    wheelAvailability.fill(true);  // default: all wheels available (matches the default AVAILABLE payload)
    if (this->numRW > 0U) {
        const RWSpeedMsgF32Payload speeds = this->rwSpeedsInMsg();
        wheelSpeeds = cArrayToEigenVector(speeds.wheelSpeeds);
        if (this->rwAvailInMsg.isLinked()) {
            const RWAvailabilityMsgPayload avail = this->rwAvailInMsg();
            for (uint32_t i = 0U; i < RW_EFF_CNT; ++i) {
                wheelAvailability[i] = (avail.wheelAvailability[i] == AVAILABLE);
            }
        }
    }

    const MrpFeedbackOutput out = this->algorithm->update(callTime, guid, wheelSpeeds, wheelAvailability);

    CmdTorqueBodyMsgF32Payload controlOut{};
    CmdTorqueBodyMsgF32Payload intFeedbackOut{};
    eigenVectorToCArray(out.controlTorque, controlOut.torqueRequestBody);
    eigenVectorToCArray(out.intFeedbackTorque, intFeedbackOut.torqueRequestBody);

    this->cmdTorqueOutMsg.write(&controlOut, this->moduleID, callTime);
    this->intFeedbackTorqueOutMsg.write(&intFeedbackOut, this->moduleID, callTime);
}
