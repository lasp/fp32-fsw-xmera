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

// The coefficient members are public, so by default SWIG wraps them with pointer typemaps that only accept an
// already wrapped array object. %naturalvar switches them to the value typemaps supplied by std_array.i,
// which accept (and return) any Python sequence.
%naturalvar GimbalToMotorAnglesSpline::motor1SplineCoefficients;
%naturalvar GimbalToMotorAnglesSpline::motor2SplineCoefficients;

%include "gimbalToMotorAnglesSplineAlgorithm.h"
%include "gimbalToMotorAnglesSpline.h"

%include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
%include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
