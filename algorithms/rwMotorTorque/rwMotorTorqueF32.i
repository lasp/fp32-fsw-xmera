%module rwMotorTorqueF32
%{
   #include "rwMotorTorque.h"
%}

%include <attribute.i>
%attribute(RwMotorTorque, Eigen::Matrix3f, controlAxes_B, getControlAxes, setControlAxes)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "rwMotorTorque.h"
%include "rwMotorTorqueAlgorithm.h"

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
%include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"

%include <fswAlgorithms/fswUtilities/fswDefinitions.h>
