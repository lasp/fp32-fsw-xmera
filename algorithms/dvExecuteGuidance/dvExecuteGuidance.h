#ifndef F32XMERA_DV_EXECUTE_GUIDANCE_H
#define F32XMERA_DV_EXECUTE_GUIDANCE_H

#include "dvExecuteGuidanceAlgorithm.h"
#include "msgPayloadDef/DvBurnCmdMsgF32Payload.h"
#include "msgPayloadDef/DvExecutionDataMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Top level structure for the execution of a Delta-V maneuver */
class DvExecuteGuidance final : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reconfigure();   //!< push edited properties into the algorithm
    void reInitialize();  //!< state-transition hook (pass-through)

    // Phase 1: Public config properties — set before reset()
    float minTime = 0.0F;              /*!< [s] Minimum burn time allowed to elapse */
    float maxTime = 0.0F;              /*!< [s] Maximum burn time; 0 disables the maximum-time criterion */
    float defaultControlPeriod = 2.0F; /*!< [s] Control period used for the first call */

    ReadFunctor<NavTransMsgF32Payload>
        navDataInMsg; /*!< [-] navigation input message that includes dv accumulation info */
    ReadFunctor<DvBurnCmdMsgF32Payload> burnDataInMsg;    /*!< [-] commanded burn input message */
    Message<THRArrayOnTimeCmdMsgF32Payload> thrCmdOutMsg; /*!< [-] thruster command on time output message */
    Message<DvExecutionDataMsgF32Payload> burnExecOutMsg; /*!< [-] burn execution output message */

   private:
    DvExecuteGuidanceConfig toConfig() const;  //!< single source of truth for reset() + reconfigure()
    std::unique_ptr<DvExecuteGuidanceAlgorithm> algorithm = nullptr;
};

#endif
