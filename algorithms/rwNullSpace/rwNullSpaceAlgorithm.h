#ifndef F32XIMERA_RW_NULL_SPACE_ALGORITHM_H
#define F32XIMERA_RW_NULL_SPACE_ALGORITHM_H

#include "msgPayloadDef/RWConstellationMsgF32Payload.h"
#include "msgPayloadDef/RWSpeedMsgF32Payload.h"
#include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"

#include <Eigen/Core>

#include <stdint.h>
#include <stdlib.h>

/*! @brief The configuration structure for the rwNullSpace module.  */
class RwNullSpaceAlgorithm {
   public:
    void reset(RWConstellationMsgF32Payload& rwConfigInMsg);
    RwMotorTorqueMsgF32Payload update(RwMotorTorqueMsgF32Payload& controlRequest,
                                      RWSpeedMsgF32Payload& rwSpeeds,
                                      RWSpeedMsgF32Payload& rwDesiredSpeeds) const;

    void setOmegaGain(float gain);
    float getOmegaGain() const;

   private:
    float omegaGain{};                                   //!< [-] The gain factor applied to the RW speeds
    Eigen::Matrix<float, RW_EFF_CNT, RW_EFF_CNT> tau{};  //!< [-] RW nullspace project matrix
    uint32_t numWheels{};                                //!< [-] The number of reaction wheels we have
};

#endif
