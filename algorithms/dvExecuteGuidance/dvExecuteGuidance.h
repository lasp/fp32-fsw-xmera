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

/*! @brief Top level structure for the execution of a Delta-V maneuver */
class DvExecuteGuidance : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setMinTime(float minTime);
    void setMaxTime(float maxTime);
    void setDefaultControlPeriod(float defaultControlPeriod);
    float getMinTime() const;
    float getMaxTime() const;
    float getDefaultControlPeriod() const;

    ReadFunctor<NavTransMsgF32Payload>
        navDataInMsg; /*!< [-] navigation input message that includes dv accumulation info */
    ReadFunctor<DvBurnCmdMsgF32Payload> burnDataInMsg;    /*!< [-] commanded burn input message */
    Message<THRArrayOnTimeCmdMsgF32Payload> thrCmdOutMsg; /*!< [-] thruster command on time output message */
    Message<DvExecutionDataMsgF32Payload> burnExecOutMsg; /*!< [-] burn execution output message */

   private:
    void rebuildAlgorithmConfig();

    float minTime{};                   ///< [s] minimum burn time allowed to elapse before completion
    float maxTime{};                   ///< [s] maximum burn time; 0 disables the maximum-time criterion
    float defaultControlPeriod{2.0F};  ///< [s] control period used for the first call

    DvExecuteGuidanceAlgorithm algorithm{DvExecuteGuidanceConfig::create(0.0F, 0.0F, 2.0F)};
};

#endif
