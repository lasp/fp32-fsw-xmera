%module regionsOfInterestPruneF32
%{
   #include "regionsOfInterestPrune.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <architecture/_GeneralModuleFiles/swig_eigen.i>
%include <stdint.i>
%include <carrays.i>
%array_functions(uint16_t, uint16Array)

%include <std_array.i>
struct RoiCandidateEntry;
%template(RoiCandidateEntryArray) std::array<RoiCandidateEntry, 4>;  // must match ROI_CANDIDATES_MAX

%include <architecture/_GeneralModuleFiles/sys_model.i>
// NOTE: this module has no %exception mapping. An invalid config (centralPoint.x outside [0, MAX_IMAGE_COLS],
// centralPoint.y outside [0, MAX_IMAGE_ROWS], alphaAreaPruneScaling above 2 or NaN) raises a RuntimeError when applied through reset()
// (the simulation's InitializeSimulation catches it), but terminates the Python process when passed to
// RegionsOfInterestPruneConfig::create() or reconfigure() called directly. Validate first with
// RegionsOfInterestPruneConfig.isValidCentralPointX()/isValidCentralPointY()/isValidAlphaAreaPruneScaling().
%include "regionsOfInterestPrune.h"
%include "regionsOfInterestPruneAlgorithm.h"

STRUCTASLIST(FpgaBinRegionF32)
%include "msgPayloadDef/FpgaBinGridMsgF32Payload.h"
ARRAYINTASLIST(unsigned short)
%include "msgPayloadDef/FpgaRowColSumMsgF32Payload.h"
%include "msgPayloadDef/RegionsIdentifiedMsgF32Payload.h"
