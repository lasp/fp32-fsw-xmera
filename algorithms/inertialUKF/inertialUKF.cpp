#include "inertialUKF.h"

#include <utilities/fsw/eigenSupport.h>
#include <stdexcept>

void InertialUKF::reset(uint64_t callTime) {
    if (!this->stAttInMsg.isLinked()) {
        throw std::invalid_argument("InertialUKF.stAttInMsg is unlinked");
    }
}

void InertialUKF::updateState(uint64_t callTime) {
    // Read all input messages
    STAttMsgF32Payload stAttIn = this->stAttInMsg();

    // Convert message payloads to algorithm-level structs
    STAttInput stAtt{};
    stAtt.timeTag = stAttIn.timeTag;
    stAtt.MRP_BdyInrtl = cArrayToEigenVector3(stAttIn.MRP_BdyInrtl);
    stAtt.omega_BN_B = cArrayToEigenVector3(stAttIn.omega_BN_B);

    GyroInput gyro{};
    if (this->gyrBuffInMsg.isLinked() && this->gyrBuffInMsg.isWritten()) {
        auto [accPkts] = this->gyrBuffInMsg();
        if (MAX_ACC_BUF_PKT > 0) {
            gyro.gyro_B = cArrayToEigenVector3(accPkts[0].gyro_B);
        }
    }

    RWSpeedsInput rwSpeeds{};
    if (this->rwSpeedsInMsg.isLinked() && this->rwSpeedsInMsg.isWritten()) {
        const RWSpeedMsgF32Payload rwIn = this->rwSpeedsInMsg();
        for (int i = 0; i < 4; i++) {
            rwSpeeds.wheelSpeeds[i] = rwIn.wheelSpeeds[i];
        }
    }

    RWArrayConfigInput rwConfig{};
    if (this->rwParamsInMsg.isLinked() && this->rwParamsInMsg.isWritten()) {
        const RWArrayConfigMsgF32Payload rwCfgIn = this->rwParamsInMsg();
        rwConfig.numRW = rwCfgIn.numRW;
    }

    VehicleConfigInput vehConfig{};
    if (this->massPropsInMsg.isLinked() && this->massPropsInMsg.isWritten()) {
        const VehicleConfigMsgF32Payload vehIn = this->massPropsInMsg();
        vehConfig.ISCPntB_B = Eigen::Map<const Eigen::Matrix3f>(vehIn.ISCPntB_B);
        vehConfig.massSC = vehIn.massSC;
    }

    // Call algorithm
    const InertialUKFOutput output = InertialUKFAlgorithm::updateState(stAtt, gyro, rwSpeeds, rwConfig, vehConfig);

    // Convert algorithm output to message payloads
    NavAttMsgF32Payload navAttOut{};
    navAttOut.timeTag = output.navAtt.timeTag;
    eigenVectorToCArray(output.navAtt.sigma_BN, navAttOut.sigma_BN);
    eigenVectorToCArray(output.navAtt.omega_BN_B, navAttOut.omega_BN_B);
    eigenVectorToCArray(output.navAtt.vehSunPntBdy, navAttOut.vehSunPntBdy);
    this->navStateOutMsg.write(&navAttOut, this->moduleID, callTime);

    InertialFilterMsgF32Payload filtOut{};
    filtOut.timeTag = output.filter.timeTag;
    filtOut.numObs = output.filter.numObs;
    this->filtDataOutMsg.write(&filtOut, this->moduleID, callTime);
}
