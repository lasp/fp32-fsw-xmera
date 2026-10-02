#ifndef F32XMERA_DV_MANEUVER_H
#define F32XMERA_DV_MANEUVER_H

#include "dvManeuverAlgorithm.h"
#include "msgPayloadDef/DvBurnCmdMsgF32Payload.h"
#include "msgPayloadDef/DvExecutionDataMsgF32Payload.h"
#include "msgPayloadDef/NavTransMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>

#include <stdint.h>
#include <memory>

/*! @brief Adapter for the delta-V burn execution algorithm. */
class DvManeuver final : public SysModel {
   public:
    DvManeuver() = default;
    ~DvManeuver() override = default;

    void reset(uint64_t callTime) override;
    void updateState(uint64_t callTime) override;
    void reconfigure();   //!< push edited properties into the initialized algorithm
    void reInitialize();  //!< State-transition hook; resets the algorithm internal states

    // Phase 1: Public config properties — set before reset()
    float minTime = 0.0F;       /*!< [s] Minimum burn time allowed to elapse */
    float maxTime = 0.0F;       /*!< [s] Maximum burn time; must be set to a positive value before reset() */
    float controlPeriod = 0.0F; /*!< [s] Control period (FSW time step); must be set > 0 before reset() */

    // Input messages
    ReadFunctor<NavTransMsgF32Payload>
        navDataInMsg; /*!< [-] navigation input message that includes dv accumulation info */
    ReadFunctor<DvBurnCmdMsgF32Payload> burnDataInMsg; /*!< [-] commanded burn input message */

    // Output messages
    Message<THRArrayOnTimeCmdMsgF32Payload> thrCmdOutMsg; /*!< [-] thruster command on time output message */
    Message<DvExecutionDataMsgF32Payload> burnExecOutMsg; /*!< [-] burn execution output message */

   private:
    DvManeuverConfig toConfig() const;  //!< single source of truth for reset() + reconfigure()
    std::unique_ptr<DvManeuverAlgorithm> algorithm = nullptr;
};

#endif
