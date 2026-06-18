#include "mrpSteering.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <algorithm>
#include <optional>
#include <stdexcept>

/*! @brief Validate that the required input messages are linked, build the algorithm's configuration from
 the adapter's stored properties (reading the spacecraft inertia from vehConfigInMsg and the optional RW
 array configuration from rwParamsInMsg), and (re)construct the embedded algorithm with a zero integral
 state.
 @param callTime The clock time at which the function was called (nanoseconds).
*/
void MrpSteering::reset(const uint64_t callTime) {
    // make sure optional msg connections are correctly done
    if (this->rwParamsInMsg.isLinked() && !this->rwSpeedsInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.rwSpeedsInMsg wasn't connected while rwParamsInMsg was connected.");
    }
    if (!this->guidInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.guidInMsg wasn't connected.");
    }
    if (!this->vehConfigInMsg.isLinked()) {
        throw std::invalid_argument("mrpSteering.vehConfigInMsg wasn't connected.");
    }

    const Eigen::Matrix3f inertia = cArrayToEigenMatrix3(this->vehConfigInMsg().ISCPntB_B);

    std::optional<InputRwData> rwConfiguration;
    if (this->rwParamsInMsg.isLinked()) {
        const RWArrayConfigMsgF32Payload rwConfigParams = this->rwParamsInMsg();
        InputRwData rwData{};
        rwData.GsMatrix_B = cArrayToEigenMatrix<float, 3, RW_EFF_CNT>(rwConfigParams.GsMatrix_B);
        std::copy(std::begin(rwConfigParams.JsList), std::end(rwConfigParams.JsList), std::begin(rwData.JsList));
        rwData.numRW = static_cast<uint32_t>(rwConfigParams.numRW);
        if (this->rwAvailInMsg.isLinked()) {
            const RWAvailabilityMsgPayload wheelAvailabilityMsg = this->rwAvailInMsg();
            std::copy(std::begin(wheelAvailabilityMsg.wheelAvailability),
                      std::end(wheelAvailabilityMsg.wheelAvailability),
                      std::begin(rwData.wheelAvailability));
        }
        rwConfiguration = rwData;
    }

    const MrpSteeringControlParameters controlParameters{
        .K1 = this->K1,
        .K3 = this->K3,
        .omegaMax = this->omegaMax,
        .ignoreOuterLoopFeedforward = this->ignoreOuterLoopFeedforward,
        .P = this->P,
        .Ki = this->Ki,
        .integralLimit = this->integralLimit,
        .controlPeriod = this->controlPeriod,
    };

    const MrpSteeringConfig config =
        MrpSteeringConfig::create(controlParameters, this->knownTorquePntB_B, inertia, rwConfiguration);
    this->algorithm = std::make_unique<MrpSteeringAlgorithm>(config);
}

void MrpSteering::reInitialize() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpSteering reset() has not been called.");
    }
    this->algorithm->reInitialize();
}

/*! @brief Read the guidance and (optional) reaction-wheel messages, run the steering control law, and
 write the commanded body torque output message.
 @param callTime The clock time at which the function was called (nanoseconds).
 */
void MrpSteering::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("MrpSteering reset() has not been called.");
    }

    const AttGuidMsgF32Payload guidCmdMsg = this->guidInMsg();
    const InputGuidanceData attGuidInputData{
        cArrayToEigenVector(guidCmdMsg.sigma_BR),
        cArrayToEigenVector(guidCmdMsg.omega_BR_B),
        cArrayToEigenVector(guidCmdMsg.omega_RN_B),
        cArrayToEigenVector(guidCmdMsg.domega_RN_B),
    };

    std::array<float, RW_EFF_CNT> wheelSpeeds{};
    if (this->rwParamsInMsg.isLinked()) {
        const RWSpeedMsgF32Payload wheelSpeedsMsg = this->rwSpeedsInMsg();
        std::copy(
            std::begin(wheelSpeedsMsg.wheelSpeeds), std::end(wheelSpeedsMsg.wheelSpeeds), std::begin(wheelSpeeds));
    }

    const Eigen::Vector3f Lr = this->algorithm->update(attGuidInputData, wheelSpeeds);

    CmdTorqueBodyMsgF32Payload controlOut{};
    eigenVectorToCArray(Lr, controlOut.torqueRequestBody);
    this->cmdTorqueOutMsg.write(controlOut, this->moduleID, callTime);
}
