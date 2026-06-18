%module celestialTwoBodyPointF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "celestialTwoBodyPoint.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "celestialTwoBodyPointAlgorithm.h"
%include "celestialTwoBodyPoint.h"

%include "msgPayloadDef/EphemerisMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
