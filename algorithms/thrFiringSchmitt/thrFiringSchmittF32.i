%module thrFiringSchmittF32
%{
   #include "thrFiringSchmitt.h"
   typedef std::array<float, 2> FloatArray2;
%}

%include <attribute.i>
%attribute(ThrFiringSchmitt, float, thrMinFireTime, getThrMinFireTime, setThrMinFireTime)
%attribute(ThrFiringSchmitt, ThrustPulsingRegime, thrustPulsingRegime, getThrustPulsingRegime, setThrustPulsingRegime)
%attribute(ThrFiringSchmitt, float, controlPeriod, getControlPeriod, setControlPeriod)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <std_array.i>
%template(FloatArray2) std::array<float, 2>;

%include "thrFiringSchmittAlgorithm.h"
%include "thrFiringSchmitt.h"

%include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
%include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
