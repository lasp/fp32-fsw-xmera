#ifndef F32XMERA_ATT_TRACKING_ERROR_H
#define F32XMERA_ATT_TRACKING_ERROR_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>

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

    Message<AttGuidMsgPayload> attGuidOutMsg;  //!< Output attitude guidance message

    ReadFunctor<NavAttMsgPayload> attNavInMsg;  //!< Input msg measured attitude
    ReadFunctor<AttRefMsgPayload> attRefInMsg;  //!< Input msg of reference attitude

   private:
    AttTrackingErrorAlgorithm algorithm{};
};

#endif
