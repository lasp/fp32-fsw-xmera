// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module flybyPointF32
%{
   #include "flybyPoint.h"
%}

%include <std_string.i>

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "flybyPoint.h"

%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/FlybyDiagnosticMsgPayload.h>
