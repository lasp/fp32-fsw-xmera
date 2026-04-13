/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module attTrackingErrorF32
%{
   #include "attTrackingError.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "attTrackingError.h"

%include <msgPayloadDef/NavAttMsgF32Payload.h>
%include <msgPayloadDef/AttGuidMsgF32Payload.h>
%include <msgPayloadDef/AttRefMsgF32Payload.h>
