#ifndef F32XMERA_AXIS_TO_GIMBAL_ANGLES_H
#define F32XMERA_AXIS_TO_GIMBAL_ANGLES_H

#include "axisToGimbalAnglesAlgorithm.h"
#include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
#include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <stdint.h>

#include <memory>

/*! @brief Adapter for the axis to gimbal angles algorithm. Reads the commanded body-frame thrust direction,
delegates the angle solve to AxisToGimbalAnglesAlgorithm, and writes the resulting gimbal angles. */
class AxisToGimbalAngles final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    /*! Re-push the current properties into the running algorithm. */
    void reconfigure() const;

    /*! Phase 1: user-defined configuration properties, set before reset() */
    Eigen::Vector3f sigma_MB{Eigen::Vector3f::Zero()};  //!< orientation of the M frame w.r.t. the B frame; M's -z axis
                                                        //!< is the un-deflected gimbal thrust axis

    /*! module IO interfaces */
    ReadFunctor<BodyHeadingMsgF32Payload>
        thrustDirectionInMsg;  //!< input msg containing the commanded thrust direction, body frame
    Message<TwoAxisGimbalMsgF32Payload> twoAxisGimbalOutMsg;  //!< output msg containing the gimbal angles

   private:
    AxisToGimbalAnglesConfig toConfig() const;
    std::unique_ptr<AxisToGimbalAnglesAlgorithm> algorithm = nullptr;  //!< algorithm instance
};

#endif  // F32XMERA_AXIS_TO_GIMBAL_ANGLES_H
