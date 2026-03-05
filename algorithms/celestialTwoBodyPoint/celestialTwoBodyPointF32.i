/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module celestialTwoBodyPointF32
%{
   #include "celestialTwoBodyPoint.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "celestialTwoBodyPointAlgorithm.h"
%include "celestialTwoBodyPoint.h"

%include "msgPayloadDef/EphemerisMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
