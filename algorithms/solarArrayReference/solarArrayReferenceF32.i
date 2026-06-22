%module solarArrayReferenceF32
%{
   #include "solarArrayReference.h"
   typedef std::array<Eigen::Vector3f, 2> Vector3fArray2;
%}

%include <attribute.i>
%attribute(SolarArrayReference, float, alignmentThreshold, getAlignmentThreshold, setAlignmentThreshold)
%attribute(SolarArrayReference, float, specifiedArrayAngle, getSpecifiedArrayAngle, setSpecifiedArrayAngle)
%attribute(SolarArrayReference, TrackingMode, trackingMode, getTrackingMode, setTrackingMode)
%attribute(SolarArrayReference, float, offsetAngle, getOffsetAngle, setOffsetAngle)

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>

%include <std_array.i>
%template(Vector3fArray2) std::array<Eigen::Vector3f, 2>;

%include "solarArrayReferenceTypes.h"
%include "solarArrayReference.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/AttRefMsgF32Payload.h"
%include "msgPayloadDef/HingedRigidBodyMsgF32Payload.h"
%include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
