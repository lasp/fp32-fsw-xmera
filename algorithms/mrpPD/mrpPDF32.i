%module mrpPDF32
%{
   #include "mrpPD.h"
%}

%include <attribute.i>
%attribute(MrpPD, float, K, getProportionalGainK, setProportionalGainK)
%attribute(MrpPD, float, P, getDerivativeGainP, setDerivativeGainP)
%attribute(MrpPD, Eigen::Vector3f, knownTorquePntB_B, getKnownTorquePntB_B, setKnownTorquePntB_B)

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "mrpPD.h"

%include "msgPayloadDef/AttGuidMsgF32Payload.h"
%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
