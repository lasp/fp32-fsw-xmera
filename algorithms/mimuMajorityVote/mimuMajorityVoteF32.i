%module mimuMajorityVoteF32
%{
#include "utilities/fsw/timeConstants.h"
   #include "mimuMajorityVote.h"
%}

%include <attribute.i>
%attribute(MimuMajorityVote, float, omegaThreshold, getOmegaThreshold, setOmegaThreshold)
%attribute(MimuMajorityVote, uint32_t, faultPersistenceLimit, getFaultPersistenceLimit, setFaultPersistenceLimit)

%include <std_string.i>
%include <swig_conly_data.i>
%include <sys_model.i>

%include "mimuMajorityVote.h"

%include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
%include "msgPayloadDef/MimuFaultMsgPayload.h"
