/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "convertStPlatformToBody.h"

void ConvertStPlatformToBody::reset(uint64_t callTime) {
    if (!this->stSensorInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: convertStPlatformToBody.stSensorInMsg wasn't connected.");
    }
}

void ConvertStPlatformToBody::updateState(const uint64_t callTime) {
    StSensorInput sensorIn{};
    if (this->stSensorInMsg.isWritten()) {
        const STSensorMsgPayload sensorMsg = this->stSensorInMsg();
        sensorIn.timeTag = static_cast<float>(sensorMsg.timeTag);
        for (int i = 0; i < 4; i++) {
            sensorIn.qInrtl2Case[i] = static_cast<float>(sensorMsg.qInrtl2Case[i]);
        }
        for (int i = 0; i < 3; i++) {
            sensorIn.omega_CN_C[i] = static_cast<float>(sensorMsg.omega_CN_C[i]);
        }
    }

    const auto [timeTag, sigma_BN, omega_BN_B, dcm_CB] = this->algorithm.update(sensorIn);

    STAttMsgPayload attOutMsg{};
    attOutMsg.timeTag = static_cast<double>(timeTag);
    for (int i = 0; i < 3; i++) {
        attOutMsg.MRP_BdyInrtl[i] = static_cast<double>(sigma_BN[i]);
        attOutMsg.omega_BN_B[i] = static_cast<double>(omega_BN_B[i]);
    }
    for (int i = 0; i < 9; i++) {
        attOutMsg.dcm_CB[i] = static_cast<double>(dcm_CB[i]);
    }

    this->stAttOutMsg.write(&attOutMsg, this->moduleID, callTime);
}

void ConvertStPlatformToBody::setDcmCB(const Eigen::Matrix3d& dcm_CB) {
    this->algorithm.setDcmCB(dcm_CB.cast<float>());
}

const Eigen::Matrix3d ConvertStPlatformToBody::getDcmCB() const { return this->algorithm.getDcmCB().cast<double>(); }
