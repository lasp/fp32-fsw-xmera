/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module inertialUKFF32
%{
    #include "inertialUKF.h"
    #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "inertialUKF.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/InertialFilterMsgF32Payload.h"
%include "msgPayloadDef/STAttMsgF32Payload.h"
%include "msgPayloadDef/AccDataMsgF32Payload.h"
%include "msgPayloadDef/AccPktDataMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
