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

// kMaxNumCssSensors is defined in msgPayloadDef/definitions.h, which SWIG only #includes (it does not parse
// the constexpr). Map it to the macro so std::array<double, kMaxNumCssSensors> members/returns resolve to the
// DoubleArrayCss instantiation above. (kMaxNumChebyPolys needs no such mapping: it is parsed from the
// %included cssCommAlgorithm.h.)
#define kMaxNumCssSensors MAX_NUM_CSS_SENSORS

%include "cssComm.h"
%include "cssCommAlgorithm.h"

%include "msgPayloadDef/CSSArraySensorMsgF32Payload.h"
