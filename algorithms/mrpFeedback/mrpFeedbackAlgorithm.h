#ifndef F32XMERA_MRP_FEEDBACK_ALGORITHM_H
#define F32XMERA_MRP_FEEDBACK_ALGORITHM_H

#include <cstdint>

#include "mrpFeedbackTypes.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
#include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/RWAvailabilityMsgPayload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"
#include "utilities/fsw/validInertiaCheck.h"

#include <Eigen/Core>

enum class ControlLawType { NORMAL = 0, SIMPLE_INTEGRAL = 1 };

struct MrpFeedbackOutput {
    CmdTorqueBodyMsgF32Payload controlOut{};      //!< control torque output
    CmdTorqueBodyMsgF32Payload intFeedbackOut{};  //!< integral feedback torque output
};

/*! Feedback control gains, control-law variant, and update period. */
struct MrpFeedbackControlParameters {
    float K{};                                              //!< [N*m] proportional gain on the MRP error
    float P{};                                              //!< [N*m*s] rate-error feedback gain
    float Ki{};                                             //!< [N*m] integral feedback gain (0 disables the integral)
    float integralLimit{};                                  //!< [N*m*s] anti-windup clamp on the integrated MRP error
    ControlLawType controlLawType{ControlLawType::NORMAL};  //!< control-law variant
    float controlPeriod{};                                  //!< [s] time between two algorithm update calls
};

class MrpFeedbackConfig final {
   public:
    static MrpFeedbackConfig create(const MrpFeedbackControlParameters& controlParameters,
                                    const Eigen::Vector3f& knownTorquePntB_B,
                                    const Eigen::Matrix3f& ISCPntB_B) {
        if (!isValidK(controlParameters.K)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: K must be finite and >= 0");
        }
        if (!isValidP(controlParameters.P)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: P must be finite and >= 0");
        }
        if (!isValidKi(controlParameters.Ki)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: Ki must be finite and >= 0");
        }
        if (!isValidIntegralLimit(controlParameters.integralLimit)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: integralLimit must be finite and >= 0");
        }
        if (!isValidControlLawType(controlParameters.controlLawType)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: controlLawType must be NORMAL or SIMPLE_INTEGRAL");
        }
        if (!isValidControlPeriod(controlParameters.controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: controlPeriod must be finite and > 0");
        }
        if (!isValidKnownTorque(knownTorquePntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: knownTorquePntB_B must be finite");
        }
        if (!isValidInertia(ISCPntB_B)) {
            FSW_THROW_INVALID_ARGUMENT("mrpFeedback: ISCPntB_B must be a valid inertia matrix");
        }
        return {controlParameters, knownTorquePntB_B, ISCPntB_B};
    }

    static bool isValidK(float K) { return fsw::is_finite(K) && K >= 0.0F; }
    static bool isValidP(float P) { return fsw::is_finite(P) && P >= 0.0F; }
    static bool isValidKi(float Ki) { return fsw::is_finite(Ki) && Ki >= 0.0F; }
    static bool isValidIntegralLimit(float integralLimit) {
        return fsw::is_finite(integralLimit) && integralLimit >= 0.0F;
    }
    static bool isValidControlPeriod(float controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0F;
    }
    static bool isValidControlLawType(ControlLawType controlLawType) {
        return controlLawType == ControlLawType::NORMAL || controlLawType == ControlLawType::SIMPLE_INTEGRAL;
    }
    static bool isValidKnownTorque(const Eigen::Vector3f& knownTorquePntB_B) { return knownTorquePntB_B.allFinite(); }
    static bool isValidInertia(const Eigen::Matrix3f& ISCPntB_B) { return inertiaIsValid(ISCPntB_B); }

    const MrpFeedbackControlParameters& getControlParameters() const { return this->controlParameters; }
    const Eigen::Vector3f& getKnownTorquePntB_B() const { return this->knownTorquePntB_B; }
    const Eigen::Matrix3f& getSpacecraftInertia() const { return this->ISCPntB_B; }

   private:
    MrpFeedbackConfig(const MrpFeedbackControlParameters& controlParameters,
                      const Eigen::Vector3f& knownTorquePntB_B,
                      const Eigen::Matrix3f& ISCPntB_B)
        : controlParameters(controlParameters), knownTorquePntB_B(knownTorquePntB_B), ISCPntB_B(ISCPntB_B) {}

    MrpFeedbackControlParameters controlParameters;
    Eigen::Vector3f knownTorquePntB_B;
    Eigen::Matrix3f ISCPntB_B;
};

/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
class MrpFeedbackAlgorithm final {
   public:
    explicit MrpFeedbackAlgorithm(MrpFeedbackConfig config);

    void setConfig(const MrpFeedbackConfig& config);

    void reset(const RWArrayConfigMsgF32Payload& rwConfigMsg, bool rwIsLinked);
    MrpFeedbackOutput update(const AttGuidMsgF32Payload& guidCmd,
                             const RWSpeedMsgF32Payload& wheelSpeeds,
                             const RWAvailabilityMsgPayload& wheelsAvailability);

   private:
    MrpFeedbackConfig cfg;
    Eigen::Vector3f int_sigma{};                  //!< [s] integral of the MPR attitude error
    RWArrayConfigMsgF32Payload rwConfigParams{};  //!< RW config snapshot taken at reset() time
};

#endif
