#ifndef F32XIMERA_RW_MOTOR_VOLTAGE_ALGORITHM_H
#define F32XIMERA_RW_MOTOR_VOLTAGE_ALGORITHM_H

#include <stdint.h>

#include "rwMotorVoltageTypes.h"

#include <Eigen/Core>

/*! @brief module configuration message */
class RwMotorVoltageAlgorithm {
   public:
    RwMotorVoltageAlgorithm(float minVoltageMagnitude, float maxVoltageMagnitude);
    ~RwMotorVoltageAlgorithm() = default;

    void reset(const RwMotorVoltageRWConfig& rwConfig);
    RwMotorVoltageData update(uint64_t callTime,
                              RwMotorVoltageTorqueInput& torqueCmd,
                              const RwMotorVoltageAvailInput& rwAvailability,
                              const RwMotorVoltageSpeedInput& rwSpeed,
                              bool rwSpeedMsgIsLinked);

    void setVoltageRange(float minVoltageMagnitude, float maxVoltageMagnitude);
    Eigen::Vector2f getVoltageRange() const;
    void setGainK(float gain);
    float getGainK() const;

   private:
    float voltageMin{};                            /*!< [V]    minimum voltage below which the torque is zero */
    float voltageMax{};                            /*!< [V]    maximum output voltage */
    float K{};                                     /*!< [V/Nm] torque tracking gain for closed loop control.*/
    Eigen::Vector<float, RW_EFF_CNT> rwSpeedOld{}; /*!< [r/s]  the RW spin rates from the prior control step */
    uint64_t priorTime{};                          /*!< [ns]   Last time the module control was called */
    bool resetFlag{};                              /*!< []     Flag indicating that a module reset occurred */
    RwMotorVoltageRWConfig
        rwConfigParams{}; /*!< [-] struct to store message containing RW config parameters in body B frame */
};

#endif
