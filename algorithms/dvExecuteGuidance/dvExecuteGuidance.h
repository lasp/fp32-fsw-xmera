#ifndef F32XMERA_DV_EXECUTE_GUIDANCE_H
#define F32XMERA_DV_EXECUTE_GUIDANCE_H

#include "dvExecuteGuidanceAlgorithm.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/DvBurnCmdMsgPayload.h>
#include <architecture/msgPayloadDef/DvExecutionDataMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/THRArrayOnTimeCmdMsgPayload.h>

#include <stdint.h>

/*! @brief Top level structure for the execution of a Delta-V maneuver */
class DvExecuteGuidance : public SysModel {
   public:
    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;

    void setMinTime(double minTime);
    void setMaxTime(double maxTime);
    void setDefaultControlPeriod(double defaultControlPeriod);
    double getMinTime() const;
    double getMaxTime() const;
    double getDefaultControlPeriod() const;

    ReadFunctor<NavTransMsgPayload>
        navDataInMsg; /*!< [-] navigation input message that includes dv accumulation info */
    ReadFunctor<DvBurnCmdMsgPayload> burnDataInMsg;    /*!< [-] commanded burn input message */
    Message<THRArrayOnTimeCmdMsgPayload> thrCmdOutMsg; /*!< [-] thruster command on time output message */
    Message<DvExecutionDataMsgPayload> burnExecOutMsg; /*!< [-] burn execution output message */

   private:
    DvExecuteGuidanceAlgorithm algorithm;
};

#endif
