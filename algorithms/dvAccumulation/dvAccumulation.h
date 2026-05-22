#ifndef F32XMERA_DV_ACCUMULATION_H
#define F32XMERA_DV_ACCUMULATION_H

#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/utilities/bskLogging.h>

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class DVAccumulation : public SysModel {
   public:
    void updateState(uint64_t callTime) override;
    void reset(uint64_t callTime) override;

    Message<NavTransMsgF32Payload> dvAcumOutMsg;    //!< accumulated DV output message
    ReadFunctor<AccDataMsgF32Payload> accPktInMsg;  //!< [-] input accelerometer message

    uint32_t msgCount;       //!< [-] The total number of messages read from inputs
    uint32_t dvInitialized;  //!< [-] Flag indicating whether DV has been started completely
    uint64_t previousTime;   //!< [ns] The clock time associated with the previous run of algorithm
    double vehAccumDV_B[3];  //!< [m/s] The accumulated Delta_V in body frame components

    BSKLogger bskLogger{};  //!< BSK Logging
};

void dvAccumulation_QuickSort(AccPktDataMsgF32Payload* A, int start, int end);

#endif
