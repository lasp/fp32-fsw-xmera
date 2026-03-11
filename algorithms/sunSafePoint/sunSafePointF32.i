%module sunSafePointF32
%{
   #include "sunSafePoint.h"
%}

%include <attribute.i>
%attribute(SunSafePoint, float, minUnitMag, getMinUnitMag, setMinUnitMag)
%attribute(SunSafePoint, float, smallAngle, getSmallAngle, setSmallAngle)
%attribute(SunSafePoint, float, sunAxisSpinRate, getSunAxisSpinRate, setSunAxisSpinRate)
%attribute(SunSafePoint, Eigen::Vector3f, omega_RN_B, getOmega_RN_B, setOmega_RN_B)
%attribute(SunSafePoint, Eigen::Vector3f, sHatBdyCmd, getSHatBdyCmd, setSHatBdyCmd)

%include "std_string.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "sys_model.i"
%include "sunSafePoint.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
struct NavAttMsgF32_C;
%include "msgPayloadDef/AttGuidMsgF32Payload.h"