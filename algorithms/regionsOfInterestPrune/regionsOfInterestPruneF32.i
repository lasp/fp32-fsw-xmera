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
// NOTE: no %exception mapping exists for fsw::invalid_argument (it derives from the freestanding
// fsw::exception, not std::exception), so an out-of-range x0/y0 (above
// MAX_IMAGE_COLS/MAX_IMAGE_ROWS) passed to RegionsOfInterestPruneConfig::create() or applied via
// reset()/reconfigure() terminates the Python process instead of raising a Python exception.
// Validate with RegionsOfInterestPruneConfig.isValidX0()/isValidY0() first.
%include "regionsOfInterestPrune.h"
%include "regionsOfInterestPruneAlgorithm.h"

STRUCTASLIST(FpgaBinRegionF32)
%include "msgPayloadDef/FpgaBinGridMsgF32Payload.h"
ARRAYINTASLIST(unsigned short)
%include "msgPayloadDef/FpgaRowColSumMsgF32Payload.h"
%include "msgPayloadDef/RegionsIdentifiedMsgF32Payload.h"
