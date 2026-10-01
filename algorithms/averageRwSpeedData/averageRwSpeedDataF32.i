%module averageRwSpeedDataF32
%{
   #include "averageRwSpeedData.h"
   #include "utilities/fsw/timeConstants.h"
%}

%include <std_string.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>

%include <sys_model.i>
%include "averageRwSpeedData.h"

STRUCTASLIST(RWSpeedMsgF32Payload)
