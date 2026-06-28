%module rwMotorTorqueF32
%{
   #include "rwMotorTorque.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <std_array.i>
%template(BoolArray3) std::array<bool, 3>;

%include "rwMotorTorque.h"

%extend RwMotorTorque {
%pythoncode %{
    @property
    def desiredControlAxes_B(self):
        return self.getDesiredControlAxes()

    @desiredControlAxes_B.setter
    def desiredControlAxes_B(self, value):
        self.setDesiredControlAxes(value)
%}
}

%include "rwMotorTorqueAlgorithm.h"

%include "msgPayloadDef/CmdTorqueBodyMsgF32Payload.h"
%include "msgPayloadDef/RwMotorTorqueMsgF32Payload.h"
%include <architecture/msgPayloadDef/RWAvailabilityMsgPayload.h>
%include "msgPayloadDef/RWArrayConfigMsgF32Payload.h"
%include "msgPayloadDef/RWSpeedMsgF32Payload.h"

%include <fswAlgorithms/fswUtilities/fswDefinitions.h>
