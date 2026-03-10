/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

%module oeStateEphemF32
%{
   #include "oeStateEphem.h"
   #include "utilities/timeConstants.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_array.i>

#define kMaxOeCoeff 20
#define kMaxOeRecords 10

%template(DoubleArray20) std::array<double, 20>;
%template(FloatArray20) std::array<float, 20>;
%include "oeStateEphem.h"

%include "msgPayloadDef/TDBVehicleClockCorrelationMsgF32Payload.h"
%include "msgPayloadDef/EphemerisMsgF32Payload.h"
