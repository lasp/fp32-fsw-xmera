%module cobConverterF32
%{
   #include "cobConverter.h"
%}

%include <attribute.i>
%attribute(CobConverter, double, radius, getRadius, setRadius)
%attribute(CobConverter, double, radiusUncertainty, getRadiusUncertainty, setRadiusUncertainty)
%attribute(CobConverter, double, numStandardDeviations, getNumStandardDeviations, setNumStandardDeviations)
%attribute(CobConverter, double, standardDeviation, getStandardDeviation, setStandardDeviation)
%attribute(CobConverter, bool, outlierDetectionEnabled, isOutlierDetectionEnabled, setOutlierDetectionEnabled)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include "cobConverter.h"
%include "cobConverterAlgorithm.h"

%include <architecture/msgPayloadDef/CameraModelMsgPayload.h>
%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCOBMsgPayload.h>
%include <architecture/msgPayloadDef/OpNavCOMMsgPayload.h>
%include <architecture/msgPayloadDef/FilterMsgPayload.h>
%include <architecture/msgPayloadDef/CobConverterDiagnosticMsgPayload.h>
