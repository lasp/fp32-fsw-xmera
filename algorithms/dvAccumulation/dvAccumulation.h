#ifndef F32XMERA_DV_ACCUMULATION_H
#define F32XMERA_DV_ACCUMULATION_H

#include "dvAccumulationAlgorithm.h"
#include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <memory>

/*! @brief SysModel adapter for DvAccumulationAlgorithm. Reads the body-frame acceleration from the
 input IMU message, runs the algorithm, and writes the accumulated body-frame Delta-V to the output
 navigation message. The adapter owns time: it tags the output message with the module call time. */
class DvAccumulation final : public SysModel {
   public:
    void reset(uint64_t callTime) final;
    void updateState(uint64_t callTime) final;
    void reconfigure();                         //!< push edited properties into the algorithm
    void reInitialize();                        //!< Reset all algorithm state (state-transition hook)
    void reInitializeExceptPersistentStates();  //!< Reset only non-persistent algorithm state

    // Phase 1: Public config properties — set before reset()
    float controlPeriod = 0.0F;  //!< [s] control period (FSW time step); must be set > 0 before reset()

    Message<NavTransMsgF32Payload> dvAccumulationOutMsg;  //!< accumulated DV output message
    ReadFunctor<IMUSensorBodyMsgF32Payload> imuInMsg;     //!< [-] input IMU body message

   private:
    DvAccumulationConfig toConfig() const;  //!< single source of truth for reset() + reconfigure()
    std::unique_ptr<DvAccumulationAlgorithm> algorithm = nullptr;
};

#endif
