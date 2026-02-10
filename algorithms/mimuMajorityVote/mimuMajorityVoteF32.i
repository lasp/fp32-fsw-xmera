%module mimuMajorityVoteF32
%{
   #include "mimuMajorityVote.h"
%}

%include <attribute.i>
%attribute(MimuMajorityVote, float, omegaThreshold, getOmegaThreshold, setOmegaThreshold)

%include <std_string.i>
%include <swig_conly_data.i>
%include <sys_model.i>

%include "mimuMajorityVote.h"

%include "msgPayloadDef/IMUSensorBodyMsgF32Payload.h"
%include "msgPayloadDef/MimuFaultMsgPayload.h"
