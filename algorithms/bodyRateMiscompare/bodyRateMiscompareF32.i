%module bodyRateMiscompareF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "bodyRateMiscompare.h"
%}

%include <attribute.i>
%attribute(BodyRateMiscompare, double, bodyRateThreshold, getBodyRateThreshold, setBodyRateThreshold)
%attribute(BodyRateMiscompare, uint32_t, faultPersistenceLimit, getFaultPersistenceLimit, setFaultPersistenceLimit)
%attribute(BodyRateMiscompare, bool, useImuRates, getUseImuRates, setUseImuRates)

%include <std_string.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>

%include <sys_model.i>
%include "bodyRateMiscompare.h"

%include <architecture/msgPayloadDef/STAttMsgPayload.h>
%include "msgPayloadDef/BodyRateFaultMsgPayload.h"
%include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
