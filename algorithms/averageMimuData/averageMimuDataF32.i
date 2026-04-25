%module averageMimuDataF32
%{
   #include "averageMimuData.h"
   #include "utilities/fsw/timeConstants.h"
%}

%include <std_string.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>

%include <sys_model.i>
%include "averageMimuData.h"

%include "msgPayloadDef/MimuPacketGroupF32Payload.h"
%include "msgPayloadDef/MimuPacketF32Payload.h"
