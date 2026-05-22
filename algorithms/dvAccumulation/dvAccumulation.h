#ifndef F32XMERA_DV_ACCUMULATION_H
#define F32XMERA_DV_ACCUMULATION_H

#include "dvAccumulationAlgorithm.h"
#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <memory>

/*! @brief SysModel adapter for DvAccumulationAlgorithm. Reads the input accelerometer-packet
 message, runs the algorithm, and writes the accumulated body-frame Delta-V to the output
 navigation message. */
class DvAccumulation final : public SysModel {
   public:
    void reset(uint64_t callTime) final;
    void updateState(uint64_t callTime) final;

    Message<NavTransMsgF32Payload> dvAcumOutMsg;    //!< accumulated DV output message
    ReadFunctor<AccDataMsgF32Payload> accPktInMsg;  //!< [-] input accelerometer message

   private:
    std::unique_ptr<DvAccumulationAlgorithm> algorithm = nullptr;
};

#endif
