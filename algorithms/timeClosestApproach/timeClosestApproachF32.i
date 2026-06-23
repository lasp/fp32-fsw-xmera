// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module timeClosestApproachF32
%{
   #include "timeClosestApproach.h"
%}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "timeClosestApproach.h"
%include "timeClosestApproachAlgorithm.h"

%include "msgPayloadDef/FilterMsgF32Payload.h"
%include "msgPayloadDef/TimeClosestApproachMsgF32Payload.h"
