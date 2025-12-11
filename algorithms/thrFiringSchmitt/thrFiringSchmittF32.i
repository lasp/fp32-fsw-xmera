%module thrFiringSchmittF32
%{
   #include "thrFiringSchmitt.h"
%}

%include <attribute.i>
%attribute(ThrFiringSchmitt, float, levelOn, getLevelOn, setLevelOn)
%attribute(ThrFiringSchmitt, float, levelOff, getLevelOff, setLevelOff)
%attribute(ThrFiringSchmitt, float, thrMinFireTime, getThrMinFireTime, setThrMinFireTime)
%attribute(ThrFiringSchmitt, uint32_t, baseThrustState, getBaseThrustState, setBaseThrustState)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "thrFiringSchmitt.h"

%include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
%include "msgPayloadDef/THRArrayOnTimeCmdMsgF32Payload.h"
