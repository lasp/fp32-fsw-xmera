%module thrustMappingF32
%{
    #include "thrustMapping.h"
%}

%include <std_array.i>
%template(BoolArray6) std::array<bool, 6>;

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "thrustMapping.h"

%extend ThrustMapping {
%pythoncode %{
    @property
    def desiredControlAxes_B(self):
        return self.getDesiredControlAxes()

    @desiredControlAxes_B.setter
    def desiredControlAxes_B(self, value):
        self.setDesiredControlAxes(value)
%}
}

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/CmdForceBodyMsgF32Payload.h"
%include "msgPayloadDef/THRArrayAvailabilityMsgF32Payload.h"
%include "msgPayloadDef/THRArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/VehicleConfigMsgF32Payload.h"
%include "msgPayloadDef/THRArrayCmdForceMsgF32Payload.h"
