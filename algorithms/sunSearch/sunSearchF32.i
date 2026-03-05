/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module sunSearchF32
%{
   #include "sunSearch.h"
   #include "utilities/timeConstants.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>

STRUCTASLIST(SlewProperties)

%include "sunSearch.h"
%include "sunSearchAlgorithm.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/AttGuidMsgF32Payload.h"
