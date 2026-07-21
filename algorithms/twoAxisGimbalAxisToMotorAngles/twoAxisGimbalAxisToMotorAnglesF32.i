// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module twoAxisGimbalAxisToMotorAnglesF32
%{
    #include "twoAxisGimbalAxisToMotorAngles.h"
%}

%include <std_string.i>
%include <std_array.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>
%include <sys_model.i>

%include "twoAxisGimbalAxisToMotorAnglesTypes.h"

%template(GimbalMotorTableRow) std::array<float, 76>;
%template(GimbalMotorTable2D) std::array<std::array<float, 76>, 111>;

%include "twoAxisGimbalAxisToMotorAnglesAlgorithm.h"
%include "twoAxisGimbalAxisToMotorAngles.h"

%include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
