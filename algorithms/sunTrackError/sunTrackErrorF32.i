%module sunTrackErrorF32
%{
   #include "sunTrackError.h"
%}

%include <attribute.i>
%attribute(SunTrackError, Eigen::Vector3f, sigma_R0R, getSigma_R0R, setSigma_R0R)
%attribute(SunTrackError, Eigen::Vector3f, sensitiveHat_B, getSensitiveHat_B, setSensitiveHat_B)
%attribute(SunTrackError, float, angleRate, getAngleRate, setAngleRate)

%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>

%include "sunTrackError.h"
%include "sunTrackErrorAlgorithm.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/AttGuidMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"
