%module centerOfBrightnessF32
%{
   #include <memory>
   #include "centerOfBrightness.h"
   #include "imageReaderInterface.h"
   #include "_imageReader/imageReaderFromFile.h"
   #include "_imageReader/imageReaderFromMessage.h"
%}

%include <stdint.i>
%include <std_string.i>
%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_array.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <std_shared_ptr.i>

%include <attribute.i>
%attribute(CenterOfBrightness, float, relativeBrightnessIncreaseThreshold, getRelativeBrightnessIncreaseThreshold,
           setRelativeBrightnessIncreaseThreshold)
%attribute(CenterOfBrightness, int32_t, numberOfPointsBrightnessAverage, getNumberOfPointsBrightnessAverage,
           setNumberOfPointsBrightnessAverage)
%attribute(CenterOfBrightness, int32_t, cameraID, getCameraID, setCameraID)

%shared_ptr(ImageReaderInterface)
%shared_ptr(ImageReaderFromFile)
%shared_ptr(ImageReaderFromMessage)

%include "imageReaderInterface.h"
%include "_imageReader/imageReaderFromFile.h"
%include "_imageReader/imageReaderFromMessage.h"

%include "centerOfBrightness.h"

%include "msgPayloadDef/CameraImageMsgF32Payload.h"
%include "msgPayloadDef/RegionOfInterestMsgF32Payload.h"
%include "msgPayloadDef/OpNavCOBMsgF32Payload.h"
%include "msgPayloadDef/CenterOfBrightnessDiagnosticMsgF32Payload.h"
