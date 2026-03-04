%module ephemNavConverterF32
%{
   #include "ephemNavConverter.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "ephemNavConverter.h"
%include "ephemNavConverterAlgorithm.h"

%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"
