%module thrustAxisToMotorAnglesF32
%{
    #include "thrustAxisToMotorAngles.h"
%}

%include <std_string.i>
%include <std_array.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>
%include <sys_model.i>

%include "thrustAxisToMotorAnglesTypes.h"

%template(GimbalMotorTableRow) std::array<float, 74>;
%template(GimbalMotorTable2D) std::array<std::array<float, 74>, 109>;

%include "thrustAxisToMotorAnglesAlgorithm.h"
%include "thrustAxisToMotorAngles.h"

%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
