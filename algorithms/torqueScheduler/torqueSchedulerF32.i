// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module torqueSchedulerF32
%{
   #include "torqueScheduler.h"
%}

%include <architecture/_GeneralModuleFiles/sys_model.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include "torqueSchedulerTypes.h"
%include "torqueSchedulerAlgorithm.h"
%include "torqueScheduler.h"

%include <architecture/msgPayloadDef/ArrayMotorTorqueMsgPayload.h>
%include <architecture/msgPayloadDef/ArrayEffectorLockMsgPayload.h>
