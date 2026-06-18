%module thrFiringRemainderF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "thrFiringRemainder.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <attribute.i>
%attribute(ThrFiringRemainder, double, thrMinFireTime, getThrMinFireTime, setThrMinFireTime)
%attribute(ThrFiringRemainder, ThrustPulsingRegime, thrustPulsingRegime, getThrustPulsingRegime, setThrustPulsingRegime)
%attribute(ThrFiringRemainder, double, controlPeriod, getControlPeriod, setControlPeriod)
%attribute(ThrFiringRemainder, double, onTimeSaturationFactor, getOnTimeSaturationFactor, setOnTimeSaturationFactor)

%include "thrFiringRemainderTypes.h"
%include "thrFiringRemainderAlgorithm.h"
%include "thrFiringRemainder.h"

%include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
%include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
