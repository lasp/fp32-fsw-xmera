%module oeStateEphemF32
%{
   #include "oeStateEphem.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_array.i>

#define kMaxOeCoeff 20
#define kMaxOeRecords 10

%include "oeStateEphemTypes.h"

%template(DoubleArray20) std::array<double, 20>;
%template(FloatArray20) std::array<float, 20>;
%include "oeStateEphem.h"

%include <architecture/msgPayloadDef/TDBVehicleClockCorrelationMsgPayload.h>
%include "msgPayloadDef/EphemerisMsgF32Payload.h"
