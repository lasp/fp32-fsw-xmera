// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module twoAxisGimbalAxisToMotorAngles
%{
    #include "twoAxisGimbalAxisToMotorAngles.h"
%}

%include <std_string.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>
%include <sys_model.i>

%include "twoAxisGimbalAxisToMotorAngles.h"

%include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
%include <architecture/msgPayloadDef/TwoAxisGimbalMsgPayload.h>
