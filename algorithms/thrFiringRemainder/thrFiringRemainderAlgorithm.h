/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

#ifndef THR_FIRING_REMAINDER_ALGORITHM
#define THR_FIRING_REMAINDER_ALGORITHM

#include "thrFiringRemainderTypes.h"

#include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
#include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
#include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"

#include <stdint.h>
#include <array>

class ThrFiringRemainderAlgorithm {
   public:
    void reset(const THRArrayConfigMsgF32Payload& thrConfigInMsgPayload);
    THRArrayOnTimeCmdMsgF32Payload update(uint64_t callTime, THRArrayCmdForceMsgF32Payload thrForceInMsgPayload);

    void setThrMinFireTime(float thrMinFireTime);
    float getThrMinFireTime() const;

    void setThrustPulsingRegime(ThrustPulsingRegime thrustPulsingRegime);
    ThrustPulsingRegime getThrustPulsingRegime() const;

    void setDefaultControlPeriod(float defaultControlPeriod);
    float getDefaultControlPeriod() const;

   private:
    uint64_t prevCallTime{};                           //!< callTime from previous function call
    std::array<float, MAX_EFF_CNT> pulseRemainder{};  //!< [-] Unimplemented thrust pulses (number of minimum pulses)
    float thrMinFireTime{};                           //!< [s] Minimum fire time
    uint32_t numThrusters{};                          //!< [-] The number of thrusters available on vehicle
    std::array<float, MAX_EFF_CNT> maxThrust{};       //!< [N] Max thrust
    ThrustPulsingRegime thrustPulsingRegime{};         //!< [-] Indicates on-pulsing or off-pulsing
    float defaultControlPeriod{};  //!< [s] Default control period used for first call //Setter and Getter
};

#endif
