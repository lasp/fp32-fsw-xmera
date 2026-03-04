/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "rwMotorVoltageAlgorithm.h"
#include "utilities/timeConstants.h"

#include "../freestandingInvalidArgument.h"
#include <algorithm>

/**
 * @brief Construct RwMotorVoltageAlgorithm
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
RwMotorVoltageAlgorithm::RwMotorVoltageAlgorithm(const float minVoltageMagnitude, const float maxVoltageMagnitude) {
    this->setVoltageRange(minVoltageMagnitude, maxVoltageMagnitude);
}

/*! This method performs a reset of the module as far as closed loop control is concerned.  Local module variables that
 retain time varying states between function calls are reset to their default values.
 @return void
 @param rwParamsInMsg struct to store message containing RW config parameters
 */
void RwMotorVoltageAlgorithm::reset(const RwMotorVoltageRWConfig& rwConfig) {
    /*! - Read static RW config data message and store it in module variables*/
    this->rwConfigParams = rwConfig;

    /* reset variables */
    this->rwSpeedOld.setZero();
    this->resetFlag = true;

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;
}

/*! Update performs the torque to voltage conversion. If a wheel speed message was provided, it also does closed loop
 control of the voltage sent. It then writes the voltage message.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 @param torqueCmd RW motor torque input message
 @param rwAvailability RW availability message
 @param rwSpeed RW speed message
 @param rwSpeedMsgIsLinked boolean indicating whether RW speed message is linked
 */
RwMotorVoltageData RwMotorVoltageAlgorithm::update(const uint64_t callTime,
                                                   RwMotorVoltageTorqueInput& torqueCmd,
                                                   const RwMotorVoltageAvailInput& rwAvailability,
                                                   const RwMotorVoltageSpeedInput& rwSpeed,
                                                   const bool rwSpeedMsgIsLinked) {
    RwMotorVoltageData voltageOut{};

    /* zero the output voltage vector */
    Eigen::Vector<float, RW_EFF_CNT> voltage{};
    voltage.setZero();

    /* if the torque closed-loop is on, evaluate the feedback term */
    if (rwSpeedMsgIsLinked) {
        /* make sure the clock didn't just initialize, or the module was recently reset */
        if (this->priorTime != 0) {
            const float dt =
                static_cast<float>(callTime - this->priorTime) * kNano2SecF; /*!< [s]   control update period */
            Eigen::Vector<float, RW_EFF_CNT> OmegaDot{};
            OmegaDot.setZero();
            for (int32_t i = 0; i < this->rwConfigParams.numRW; i++) {
                if (rwAvailability.wheelAvailability[i] == RW_MOTOR_VOLTAGE_AVAILABLE && !this->resetFlag) {
                    OmegaDot[i] = (rwSpeed.wheelSpeeds[i] - this->rwSpeedOld[i]) / dt;
                    torqueCmd.motorTorque[i] -=
                        this->K * (this->rwConfigParams.JsList[i] * OmegaDot[i] - torqueCmd.motorTorque[i]);
                }
                this->rwSpeedOld[i] = rwSpeed.wheelSpeeds[i];
            }
            this->resetFlag = false;
        }
        this->priorTime = callTime;
    }

    /* evaluate the feedforward mapping of torque into voltage */
    for (int32_t i = 0; i < this->rwConfigParams.numRW; ++i) {
        if (rwAvailability.wheelAvailability[i] == RW_MOTOR_VOLTAGE_AVAILABLE) {
            voltage[i] =
                (this->voltageMax - this->voltageMin) / this->rwConfigParams.uMax[i] * torqueCmd.motorTorque[i];
            if (voltage[i] > 0.0) voltage[i] += this->voltageMin;
            if (voltage[i] < 0.0) voltage[i] -= this->voltageMin;
        }
        /* check for voltage saturation */
        voltage[i] = std::clamp(voltage[i], -this->voltageMax, this->voltageMax);

        voltageOut.voltage[i] = voltage[i];
    }

    return voltageOut;
}

/**
 * @brief Set the minimum and maximum voltage.
 * @param minVoltageMagnitude minimum voltage below which the torque is zero.
 * @param maxVoltageMagnitude maximum output voltage
 */
void RwMotorVoltageAlgorithm::setVoltageRange(const float minVoltageMagnitude, const float maxVoltageMagnitude) {
    if (minVoltageMagnitude < 0.0) {
        FS_THROW_INVALID_ARGUMENT("minVoltageMagnitude must not be negative.");
    }
    if (maxVoltageMagnitude < 0.0) {
        FS_THROW_INVALID_ARGUMENT("maxVoltageMagnitude must not be negative.");
    }
    if (maxVoltageMagnitude <= minVoltageMagnitude) {
        FS_THROW_INVALID_ARGUMENT("maxVoltageMagnitude must be greater than minVoltageMagnitude.");
    }
    this->voltageMin = minVoltageMagnitude;
    this->voltageMax = maxVoltageMagnitude;
}

/**
 * @brief Get the minimum and maximum voltage.
 * @return Eigen::Vector2f minimum and maximum voltage
 */
Eigen::Vector2f RwMotorVoltageAlgorithm::getVoltageRange() const {
    return Eigen::Vector2f{this->voltageMin, this->voltageMax};
}

/**
 * @brief Set the feedback gain.
 * @param gain feedback gain.
 */
void RwMotorVoltageAlgorithm::setGainK(const float gain) {
    if (gain < 0.0) {
        FS_THROW_INVALID_ARGUMENT("Feedback gain must not be negative");
    }
    this->K = gain;
}

/**
 * @brief Get the feedback gain.
 * @return float feedback gain.
 */
float RwMotorVoltageAlgorithm::getGainK() const { return this->K; }
