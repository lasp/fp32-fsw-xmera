/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XMERA_MRP_STEERING_ALGORITHM_H
#define F32XMERA_MRP_STEERING_ALGORITHM_H

#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "msgPayloadDef/RateCmdMsgF32Payload.h"

/*! @brief Data structure for the MRP feedback attitude control routine. */
class MrpSteeringAlgorithm {
   public:
    RateCmdMsgF32Payload update(AttGuidMsgF32Payload& guidInMsg) const;

    void setK1(float gain);
    float getK1() const;
    void setK3(float gain);
    float getK3() const;
    void setOmegaMax(float omega);
    float getOmegaMax() const;
    void setIgnoreFeedforward(bool ignore);
    bool getIgnoreFeedforward() const;

   private:
    float K1{};                        //!< [rad/sec] Proportional gain applied to MRP errors
    float K3{};                        //!< [rad/sec] Cubic gain applied to MRP error in steering saturation function
    float omegaMax{};                  //!< [rad/sec] Maximum rate command of steering control
    bool ignoreOuterLoopFeedforward{};  //!< [] Boolean flag indicating if outer feedforward term should be included
};

#endif
