%module solarArrayReferenceF32
%{
   #include "solarArrayReference.h"
   #include "utilities/timeConstants.h"
   typedef std::array<Eigen::Vector3f, 2> Vector3fArray2;
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <std_array.i>
%template(Vector3fArray2) std::array<Eigen::Vector3f, 2>;

%include "solarArrayReference.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
