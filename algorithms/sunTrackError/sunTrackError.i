%module sunTrackError
%{
   #include "sunTrackError.h"
%}

%include <std_string.i>
%include <swig_eigen.i>
%include <swig_conly_data.i>
%include <sys_model.i>

%include "sunTrackError.h"

%include <architecture/msgPayloadDef/NavAttMsgPayload.h>
%include <architecture/msgPayloadDef/AttGuidMsgPayload.h>
%include <architecture/msgPayloadDef/AttRefMsgPayload.h>
%include <architecture/msgPayloadDef/NavTransMsgPayload.h>
%include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
