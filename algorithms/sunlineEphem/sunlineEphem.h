#ifndef F32XIMERA_SUNLINE_EPHEM_H
#define F32XIMERA_SUNLINE_EPHEM_H

#include "msgPayloadDef/EphemerisMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "sunlineEphemAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

class SunlineEphem : public SysModel {
   public:
    SunlineEphem() = default;
    ~SunlineEphem() = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    Message<NavAttMsgF32Payload> navStateOutMsg;           /*!< The name of the output message*/
    ReadFunctor<EphemerisMsgF32Payload> sunPositionInMsg;  //!< The name of the sun ephemeris input message
    ReadFunctor<NavTransMsgF32Payload> scPositionInMsg;    //!< The name of the spacecraft ephemeris input message
    ReadFunctor<NavAttMsgF32Payload> scAttitudeInMsg;      //!< The name of the spacecraft attitude input message

   private:
    SunlineEphemAlgorithm algorithm{};
};

#endif
