%module gimbalAxisToMotorAnglesF32
%{
    #include "gimbalAxisToMotorAngles.h"
%}

%include <std_string.i>
%include <std_array.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>
%include <sys_model.i>

%include "gimbalAxisToMotorAnglesTypes.h"

%template(GimbalMotorTableRow) std::array<float, 76>;
%template(GimbalMotorTable2D) std::array<std::array<float, 76>, 111>;

%include "gimbalAxisToMotorAnglesAlgorithm.h"
%include "gimbalAxisToMotorAngles.h"

%include "msgPayloadDef/BodyHeadingMsgF32Payload.h"
%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
