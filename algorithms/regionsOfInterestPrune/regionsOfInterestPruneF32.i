%module regionsOfInterestPruneF32
%{
   #include "regionsOfInterestPrune.h"
%}

%include <attribute.i>
%attribute(RegionsOfInterestPrune, uint32_t, maxRowSpans, getMaxRowSpans, setMaxRowSpans)
%attribute(RegionsOfInterestPrune, uint32_t, maxColSpans, getMaxColSpans, setMaxColSpans)

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "regionsOfInterestPrune.h"
%include "regionsOfInterestPruneAlgorithm.h"

%include "msgPayloadDef/FpgaRowColSumMsgF32Payload.h"
%include "msgPayloadDef/FpgaThreshImageMsgF32Payload.h"
%include "msgPayloadDef/RegionOfInterestMsgF32Payload.h"
%include "msgPayloadDef/RegionsIdentifiedMsgF32Payload.h"
