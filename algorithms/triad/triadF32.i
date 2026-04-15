// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module triadF32
%{
   #include "triad.h"
%}

%include <attribute.i>
%attribute(Triad, Eigen::Vector3d, a1Hat_B, getA1Hat_B, setA1Hat_B)
%attribute(Triad, Eigen::Vector3d, h1Hat_B, getH1Hat_B, setH1Hat_B)
%attribute(Triad, Eigen::Vector3d, hHat_N, getHHat_N, setHHat_N)
%attribute(Triad, CelestialBody, celestialBodyInput, getCelestialBodyInput, setCelestialBodyInput)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "triadTypes.h"
%include "triad.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/BodyHeadingMsgPayload.h>
%include <architecture/msgPayloadDef/InertialHeadingMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
