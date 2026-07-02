// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module triadF32
%{
   #include "triad.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "triad.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
