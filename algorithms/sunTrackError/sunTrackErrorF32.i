%module sunTrackErrorF32
%{
   #include "sunTrackError.h"
%}

%include <attribute.i>
%attribute(SunTrackError, Eigen::Vector3d, sigma_R0R, getSigma_R0R, setSigma_R0R)
%attribute(SunTrackError, Eigen::Vector3d, sensitiveHat_B, getSensitiveHat_B, setSensitiveHat_B)
%attribute(SunTrackError, double, angleRate, getAngleRate, setAngleRate)

%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>

%include "sunTrackError.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
