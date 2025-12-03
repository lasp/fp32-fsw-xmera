%module averageMimuDataF32
%{
   #include "averageMimuData.h"
%}

%include <std_string.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>

%include <sys_model.i>
%include "averageMimuData.h"

%include "msgPayloadDef/AccDataMsgF32Payload.h"
