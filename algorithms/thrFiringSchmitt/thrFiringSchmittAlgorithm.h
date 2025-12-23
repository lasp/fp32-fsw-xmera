#ifndef F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H
#define F32XMERA_THR_FIRING_SCHMITT_ALGORITHM_H

#include <stdint.h>

#include <array>

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
#include <architecture/msgPayloadDef/definitions.h>

enum class PulsingRegime { ONPULSING = 0, OFFPULSING = 1 };

class ThrFiringSchmittAlgorithm final {
   public:
    void reset(THRArrayConfigMsgF32Payload const& thrusterConfigPayload);
    THRArrayOnTimeCmdMsgF32Payload update(uint64_t callTime, THRArrayCmdForceMsgF32Payload& thrForceIn);
    std::array<float, 2U> getLevelsOnOff() const;
    void setLevelsOnOff(float levelOn, float levelOff);
    float getThrMinFireTime() const;
    void setThrMinFireTime(float time);
    PulsingRegime getBaseThrustState() const;
    void setBaseThrustState(PulsingRegime state);

   private:
    float levelOn{};                                 //!< [-] ON duty cycle fraction
    float levelOff{};                                //!< [-] OFF duty cycle fraction
    float thrMinFireTime{};                          //!< [s] Minimum ON time for thrusters
    PulsingRegime baseThrustState{};                  //!< [-] Indicates on-pulsing (0) or off-pulsing (1)
    uint32_t numThrusters{};                          //!< [-] The number of thrusters available on vehicle
    std::array<float, MAX_EFF_CNT> maxThrust{};      //!< [N] Max thrust
    std::array<bool, MAX_EFF_CNT> prevThrustState{};  //!< [-] ON/OFF state of thrusters from previous call
    uint64_t prevCallTime{};                          //!< callTime from previous function call
};

#endif
