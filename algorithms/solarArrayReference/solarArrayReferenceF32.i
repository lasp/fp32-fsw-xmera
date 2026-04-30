%module solarArrayReferenceF32
%{
   #include "solarArrayReference.h"
   #include "utilities/timeConstants.h"
%}

%include <attribute.i>
%attribute(SolarArrayReference, Eigen::Vector3d, a1Hat_B, getA1Hat_B, setA1Hat_B)
%attribute(SolarArrayReference, Eigen::Vector3d, a2Hat_B, getA2Hat_B, setA2Hat_B)
%attribute(SolarArrayReference, int, attitudeFrame, getAttitudeFrame, setAttitudeFrame)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "solarArrayReference.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/HingedRigidBodyMsgPayload.h>
