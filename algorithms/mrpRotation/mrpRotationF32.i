%module mrpRotationF32
%{
   #include "mrpRotation.h"
%}

%include <attribute.i>
%attribute(MrpRotation, Eigen::Vector3f, sigma_RR0, getSigmaRR0, setSigmaRR0)
%attribute(MrpRotation, Eigen::Vector3f, omega_RR0_R, getOmegaRR0, setOmegaRR0)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "mrpRotation.h"
%include "mrpRotationAlgorithm.h"

%include "msgPayloadDef/AttRefMsgF32Payload.h"
%include "msgPayloadDef/AttStateMsgF32Payload.h"
