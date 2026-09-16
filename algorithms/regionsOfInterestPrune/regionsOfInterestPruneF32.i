%module regionsOfInterestPruneF32
%{
   #include "regionsOfInterestPrune.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <stdint.i>
%include <carrays.i>
%array_functions(uint16_t, uint16Array)

%include <std_array.i>
struct RoiCandidateEntry;
%template(RoiCandidateEntryArray) std::array<RoiCandidateEntry, 16>;  // must match ROI_CANDIDATES_MAX

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include "regionsOfInterestPrune.h"
%include "regionsOfInterestPruneAlgorithm.h"

%include "msgPayloadDef/FpgaRowColSumMsgF32Payload.h"
%include "msgPayloadDef/RegionsIdentifiedMsgF32Payload.h"
