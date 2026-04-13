#ifndef F32XMERA_ATT_TRACKING_ERROR_H
#define F32XMERA_ATT_TRACKING_ERROR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <msgPayloadDef/AttGuidMsgF32Payload.h>
#include <msgPayloadDef/AttRefMsgF32Payload.h>
#include <msgPayloadDef/NavAttMsgF32Payload.h>

#include "attTrackingErrorAlgorithm.h"

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

    Message<AttGuidMsgF32Payload> attGuidOutMsg;  //!< Output attitude guidance message

    ReadFunctor<NavAttMsgF32Payload> attNavInMsg;  //!< Input msg measured attitude
    ReadFunctor<AttRefMsgF32Payload> attRefInMsg;  //!< Input msg of reference attitude

   private:
    AttTrackingErrorAlgorithm algorithm{};
};

#endif
