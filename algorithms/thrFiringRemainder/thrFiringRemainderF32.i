%module thrFiringRemainderF32
%{
   #include "thrFiringRemainder.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <attribute.i>
%attribute(ThrFiringRemainder, double, thrMinFireTime, getThrMinFireTime, setThrMinFireTime)
%attribute(ThrFiringRemainder, ThrustPulsingRegime, thrustPulsingRegime, getThrustPulsingRegime, setThrustPulsingRegime)
%attribute(ThrFiringRemainder, double, controlPeriod, getControlPeriod, setControlPeriod)
%attribute(ThrFiringRemainder, double, onTimeSaturationFactor, getOnTimeSaturationFactor, setOnTimeSaturationFactor)

%include "thrFiringRemainder.h"
%include "thrFiringRemainderTypes.h"

%include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
%include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
