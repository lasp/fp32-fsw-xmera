#ifndef F32XMERA_RW_MOTOR_VOLTAGE_TYPES_H
#define F32XMERA_RW_MOTOR_VOLTAGE_TYPES_H

#include "msgPayloadDef/definitions.h"

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Reaction wheel availability states for rwMotorVoltage */
enum RwMotorVoltageWheelAvailability : int32_t { RW_MOTOR_VOLTAGE_AVAILABLE = 0, RW_MOTOR_VOLTAGE_UNAVAILABLE = 1 };

/*! @brief Reaction wheel configuration data for rwMotorVoltage */
struct RwMotorVoltageRWConfig {
    float JsList[kMaxNumRw] = {};  //!< [kg m^2] Spin axis inertias for each RW
    float uMax[kMaxNumRw] = {};    //!< [Nm]     Maximum RW motor torque for each RW
    int32_t numRW = 0;             //!< [-]      Number of reaction wheels
};

/*! @brief Torque input data for rwMotorVoltage (mutable, used in closed-loop feedback) */
struct RwMotorVoltageTorqueInput {
    float motorTorque[kMaxNumRw] = {};  //!< [Nm] Motor torque array for each reaction wheel
};

/*! @brief Reaction wheel speed input data for rwMotorVoltage */
struct RwMotorVoltageSpeedInput {
    float wheelSpeeds[kMaxNumRw] = {};  //!< [rad/s] Wheel speeds for each reaction wheel
};

/*! @brief Reaction wheel availability input data for rwMotorVoltage */
struct RwMotorVoltageAvailInput {
    int32_t wheelAvailability[kMaxNumRw] = {};  //!< [-] Availability status for each reaction wheel
};

/*! @brief Reaction wheel motor voltage data produced by the algorithm */
struct RwMotorVoltageData {
    float voltage[kMaxNumRw] = {};  //!< [V] Voltage array for each reaction wheel
};

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // F32XMERA_RW_MOTOR_VOLTAGE_TYPES_H
