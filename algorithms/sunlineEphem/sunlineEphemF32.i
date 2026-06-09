%module sunlineEphemF32
%{
    #include "sunlineEphem.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "sunlineEphem.h"
%include "sunlineEphemAlgorithm.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"
