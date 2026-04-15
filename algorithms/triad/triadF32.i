// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module triadF32
%{
   #include "triad.h"
%}

%include <attribute.i>
%attribute(Triad, Eigen::Vector3f, a1Hat_B, getA1Hat_B, setA1Hat_B)
%attribute(Triad, Eigen::Vector3f, h1Hat_B, getH1Hat_B, setH1Hat_B)
%attribute(Triad, Eigen::Vector3f, hHat_N, getHHat_N, setHHat_N)
%attribute(Triad, CelestialBody, celestialBodyInput, getCelestialBodyInput, setCelestialBodyInput)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "triadTypes.h"
%include "triad.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
%include "msgPayloadDef/InertialHeadingMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
