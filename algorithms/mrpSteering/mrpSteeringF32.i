/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

%module mrpSteeringF32
%{
   #include "mrpSteering.h"
%}

%include <attribute.i>
%attribute(MrpSteering, float, K1, getK1, setK1)
%attribute(MrpSteering, float, K3, getK3, setK3)
%attribute(MrpSteering, float, omegaMax, getOmegaMax, setOmegaMax)
%attribute(MrpSteering, bool, ignoreOuterLoopFeedforward, getIgnoreFeedforward, setIgnoreFeedforward)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "mrpSteering.h"
%include "mrpSteeringAlgorithm.h"

%include "msgPayloadDef/AttGuidMsgF32Payload.h"
%include "msgPayloadDef/RateCmdMsgF32Payload.h"
