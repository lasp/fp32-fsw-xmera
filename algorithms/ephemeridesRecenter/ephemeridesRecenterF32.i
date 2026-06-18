%module ephemeridesRecenterF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "ephemeridesRecenter.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_vector.i>
%include <std_array.i>
%template(IntArray20) std::array<int, MAX_NUM_CHANGE_BODIES>;

%include "ephemeridesRecenter.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"
