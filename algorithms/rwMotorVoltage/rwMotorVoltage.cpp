#include "rwMotorVoltage.h"

#include <stdexcept>

/**
 * @brief Construct RwMotorVoltage
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
RwMotorVoltage::RwMotorVoltage(const float minVoltageMagnitude, const float maxVoltageMagnitude)
    : algorithm(minVoltageMagnitude, maxVoltageMagnitude) {}

/*! This method performs a reset of the module as far as closed loop control is concerned.  Local module variables that
 retain time varying states between function calls are reset to their default values.
 @return void
 @param callTime Sim time in nanos
 */
void RwMotorVoltage::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->rwParamsInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorVoltage.rwParamsInMsg wasn't connected.");
    }
    if (!this->torqueInMsg.isLinked()) {
        throw std::invalid_argument("rwMotorVoltage.torqueInMsg wasn't connected.");
    }

    const RWArrayConfigMsgF32Payload rwParamsMsg = this->rwParamsInMsg();
    RwMotorVoltageRWConfig rwConfig{};
    std::ranges::copy(rwParamsMsg.JsList, std::begin(rwConfig.JsList));
    std::ranges::copy(rwParamsMsg.uMax, std::begin(rwConfig.uMax));
    rwConfig.numRW = rwParamsMsg.numRW;

    this->algorithm.reset(rwConfig);
}

/*! Update performs the torque to voltage conversion. If a wheel speed message was provided, it also does closed loop
 control of the voltage sent. It then writes the voltage message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void RwMotorVoltage::updateState(uint64_t callTime) {
    /* - Read the input messages */
    const auto [motorTorque] = this->torqueInMsg();
    RwMotorVoltageTorqueInput torqueCmd{};
    std::ranges::copy(motorTorque, std::begin(torqueCmd.motorTorque));

    RwMotorVoltageSpeedInput rwSpeed{};
    RwMotorVoltageAvailInput rwAvailability{};

    bool rwSpeedMsgIsLinked{};
    if (this->rwSpeedInMsg.isLinked()) {
        const auto [wheelSpeeds, wheelThetas] = this->rwSpeedInMsg();
        std::ranges::copy(wheelSpeeds, std::begin(rwSpeed.wheelSpeeds));
        rwSpeedMsgIsLinked = true;
    }
    if (this->rwAvailInMsg.isLinked()) {
        const auto [wheelAvailability] = this->rwAvailInMsg();
        std::ranges::copy(wheelAvailability, std::begin(rwAvailability.wheelAvailability));
    }

    const auto [voltage] = this->algorithm.update(callTime, torqueCmd, rwAvailability, rwSpeed, rwSpeedMsgIsLinked);

    RwMotorVoltageMsgF32Payload voltageOut{};
    std::ranges::copy(voltage, std::begin(voltageOut.voltage));

    this->voltageOutMsg.write(&voltageOut, this->moduleID, callTime);
}

/**
 * @brief Set the minimum and maximum voltage.
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
void RwMotorVoltage::setVoltageRange(const float minVoltageMagnitude, const float maxVoltageMagnitude) {
    this->algorithm.setVoltageRange(minVoltageMagnitude, maxVoltageMagnitude);
}

/**
 * @brief Get the minimum and maximum voltage.
 * @return Eigen::Vector2f minimum and maximum voltage
 */
Eigen::Vector2f RwMotorVoltage::getVoltageRange() const { return this->algorithm.getVoltageRange(); }

/**
 * @brief Set the feedback gain.
 * @param gain feedback gain.
 */
void RwMotorVoltage::setGainK(const float gain) { this->algorithm.setGainK(gain); }

/**
 * @brief Get the feedback gain.
 * @return float feedback gain.
 */
float RwMotorVoltage::getGainK() const { return this->algorithm.getGainK(); }
