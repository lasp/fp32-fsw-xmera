/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
*/

%module attTrackingErrorF32
%{
   #include "attTrackingError.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}

%include "sys_model.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "attTrackingError.h"

%include "architecture/msgPayloadDef/NavAttMsgPayload.h"
%include "architecture/msgPayloadDef/AttGuidMsgPayload.h"
%include "architecture/msgPayloadDef/AttRefMsgPayload.h"

%pythoncode %{
    import sys
    protectAllClasses(sys.modules[__name__])
%}
