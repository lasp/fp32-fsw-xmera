#ifndef F32XMERA_MRP_FEEDBACK_ALGORITHM_H
#define F32XMERA_MRP_FEEDBACK_ALGORITHM_H

#include <cstdint>

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>

#include <Eigen/Core>

/*! structure containing the MRP feedback algorithm output */
typedef struct {
    CmdTorqueBodyMsgF32Payload controlOut;     /*!< control torque output */
    CmdTorqueBodyMsgF32Payload intFeedbackOut; /*!< integral feedback torque output */
} MrpFeedbackOutput;

enum class ControlLawType { NORMAL = 0, SIMPLE_INTEGRAL = 1 };

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedbackAlgorithm final {
   public:
    void reset(VehicleConfigMsgF32Payload vehConfigMsg, const RWArrayConfigMsgF32Payload& rwConfigMsg, bool rwIsLinked);
    MrpFeedbackOutput update(uint64_t callTime,
                             AttGuidMsgF32Payload& guidCmd,
                             const RWSpeedMsgF32Payload& wheelSpeeds,
                             const RWAvailabilityMsgPayload& wheelsAvailability);

    void setK(float gain);
    float getK() const;
    void setP(float gain);
    float getP() const;
    void setKi(float gain);
    float getKi() const;
    void setIntegralLimit(float limit);
    float getIntegralLimit() const;
    void setControlLawType(ControlLawType type);
    ControlLawType getControlLawType() const;
    void setKnownTorquePntB_B(const Eigen::Vector3f& torque);
    Eigen::Vector3f getKnownTorquePntB_B() const;

   private:
    float K{};              //!< [rad/sec] Proportional gain applied to MRP errors
    float P{};              //!< [N*m*s]   Rate error feedback gain applied
    float Ki{};             //!< [N*m]     Integration feedback error on rate error
    float integralLimit{};  //!< [N*m]     Integration limit to avoid wind-up issue
    ControlLawType controlLawType{};   //!<           Flag to choose between the two control laws available
    Eigen::Vector3f knownTorquePntB_B{
        Eigen::Vector3f::Zero()};  //!< [N*m]     known external torque in body frame vector components
    uint64_t priorTime{};          //!< [ns]      Last time the attitude control is called
    Eigen::Vector3f int_sigma{};   //!< [s] integral of the MPR attitude error
    Eigen::Matrix3f ISCPntB_B{};   //!< [kg m^2] Spacecraft Inertia
    RWArrayConfigMsgF32Payload
        rwConfigParams{};  //!< [-] struct to store message containing RW config parameters in body B frame
};

#endif
