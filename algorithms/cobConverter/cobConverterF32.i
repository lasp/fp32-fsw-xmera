%module cobConverterF32
%{
   #include "cobConverter.h"
%}

%include <attribute.i>
%attribute(CobConverter, float, radius, getRadius, setRadius)
%attribute(CobConverter, float, radiusUncertainty, getRadiusUncertainty, setRadiusUncertainty)
%attribute(CobConverter, float, numStandardDeviations, getNumStandardDeviations, setNumStandardDeviations)
%attribute(CobConverter, float, standardDeviation, getStandardDeviation, setStandardDeviation)
%attribute(CobConverter, bool, outlierDetectionEnabled, isOutlierDetectionEnabled, setOutlierDetectionEnabled)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "cobConverter.h"
%include "cobConverterAlgorithm.h"

%include "msgPayloadDef/CobConverterDiagnosticMsgF32Payload.h"
%include "msgPayloadDef/FilterMsgF32Payload.h"
%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/CameraModelMsgF32Payload.h"
%include "msgPayloadDef/OpNavCOBMsgF32Payload.h"
%include "msgPayloadDef/OpNavCOMMsgF32Payload.h"
%include "msgPayloadDef/OpNavUnitVecMsgF32Payload.h"
