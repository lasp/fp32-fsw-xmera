%module gimbalToMotorAnglesSplineF32
%{
    #include "gimbalToMotorAnglesSpline.h"
%}

%include <std_string.i>
%include <std_array.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>
%include <sys_model.i>

%include "gimbalToMotorAnglesSplineTypes.h"

%template(GimbalToMotorSplineCoefficients) std::array<float, NUM_SPLINE_COEFFS>;

// The coefficient member is public, so by default SWIG wraps it with pointer typemaps that only accept an
// already wrapped array object. %naturalvar switches it to the value typemaps supplied by std_array.i,
// which accept (and return) any Python sequence.
%naturalvar GimbalToMotorAnglesSpline::splineCoefficients;

%include "gimbalToMotorAnglesSplineAlgorithm.h"
%include "gimbalToMotorAnglesSpline.h"

%include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
%include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
