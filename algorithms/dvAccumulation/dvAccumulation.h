#ifndef F32XMERA_DV_ACCUMULATION_H
#define F32XMERA_DV_ACCUMULATION_H

#include "msgPayloadDef/AccDataMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

/*! @brief Accumulates body-frame Delta-V by integrating timestamped accelerometer packets. Sorts
 each input snapshot by measurement time, ingests only packets newer than the previously-seen
 latest time, and writes the running accumulator to the output navigation message. */
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
};

void dvAccumulation_QuickSort(AccPktDataMsgF32Payload* A, int start, int end);

#endif
