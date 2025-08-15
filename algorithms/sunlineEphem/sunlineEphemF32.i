/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module sunlineEphemF32
%{
    #include "sunlineEphem.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}

%include "sys_model.i"
%include "swig_conly_data.i"

%include "sunlineEphem.h"
%include "sunlineEphemAlgorithm.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}
