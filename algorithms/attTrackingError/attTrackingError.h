/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#ifndef F32XIMERA_ATT_TRACKING_ERROR_H
#define F32XIMERA_ATT_TRACKING_ERROR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "architecture/msgPayloadDef/AttRefMsgPayload.h"
#include "msgPayloadDef/AttGuidMsgF32Payload.h"
#include "architecture/msgPayloadDef/NavAttMsgPayload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "architecture/utilities/bskLogging.h"

#include "attTrackingErrorAlgorithm.h"

#include <Eigen/Dense>

#include <stdint.h>

/*!@brief Data structure for module to compute the attitude tracking error between the spacecraft attitude and the
 * reference.
 */
class AttTrackingError : public SysModel {
   public:
    AttTrackingError() = default;   //!< Constructor
    ~AttTrackingError() = default;  //!< Destructor
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void setSigma_R0R(const Eigen::Vector3d &sigma_R0R);
    const Eigen::Vector3d getSigma_R0R() const;

    Message<AttGuidMsgPayload> attGuidOutMsg;   //!< Output attitude guidance message
    Message<AttGuidMsgF32Payload> attGuidOutF32Msg;   //!< Output float (32 bit) attitude guidance message

    ReadFunctor<NavAttMsgPayload> attNavInMsg;  //!< Input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;  //!< Input msg of reference attitude
    BSKLogger bskLogger = {};                   //!< BSK Logging

   private:
    AttTrackingErrorAlgorithm algorithm{};
};

#endif
