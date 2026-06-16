%module cssCommF32
%{
   #include "cssComm.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_array.i>

// Use the same macro symbols as the C++ signatures in cssCommAlgorithm.h
// so SWIG's type registry matches the function-arg types to these
// template instantiations (mirrors ephemeridesRecenterF32.i pattern).
%template(DoubleArrayCss) std::array<double, MAX_NUM_CSS_SENSORS>;
%template(DoubleArrayCheby) std::array<double, MAX_NUM_CHEBY_POLYS>;

%include "cssComm.h"
%include "cssCommAlgorithm.h"

%include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
