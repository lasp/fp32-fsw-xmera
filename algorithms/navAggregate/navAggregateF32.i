%module navAggregateF32
%{
   #include "navAggregate.h"
   using namespace f32;
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

// Strip f32:: prefix from Python-visible names
%rename(InputNavAttData) f32::InputNavAttData;
%rename(InputNavTransData) f32::InputNavTransData;
%rename(AggregateOutput) f32::AggregateOutput;
%rename(NavAggregateAlgorithm) f32::NavAggregateAlgorithm;
%rename(AggregateAttInput) f32::AggregateAttInput;
%rename(AggregateTransInput) f32::AggregateTransInput;
%rename(NavAggregate) f32::NavAggregate;

STRUCTASLIST(f32::AggregateAttInput)
STRUCTASLIST(f32::AggregateTransInput)

%include "navAggregateOutput.h"
%include "navAggregateAlgorithm.h"
%include "navAggregate.h"

%include "msgPayloadDef/NavAttMsgF32Payload.h"
%include "msgPayloadDef/NavTransMsgF32Payload.h"
