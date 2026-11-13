%module gimbalAnglesToMotorAnglesTableF32
%{
    #include "gimbalAnglesToMotorAnglesTable.h"
%}

%include <std_string.i>
%include <std_array.i>
%include <swig_conly_data.i>
%include <swig_eigen.i>
%include <sys_model.i>

%include "gimbalAnglesToMotorAnglesTableTypes.h"

%template(GimbalMotorAngleTable) std::array<float, NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS>;
%template(GimbalMotorTableRowLayout) std::array<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS>;

// The table members are public, so by default SWIG wraps them with pointer typemaps that only accept an
// already wrapped array object. %naturalvar switches them to the value typemaps supplied by std_array.i,
// which accept (and return) any Python sequence.
%naturalvar GimbalToMotorAngleTableLayout::rowStartStrideIndices;
%naturalvar GimbalToMotorAngleTableLayout::rowStartColIndices;
%naturalvar GimbalAnglesToMotorAnglesTable::gimbalToMotor1AngleData;
%naturalvar GimbalAnglesToMotorAnglesTable::gimbalToMotor2AngleData;
%naturalvar GimbalAnglesToMotorAnglesTable::rowStartStrideIndices;
%naturalvar GimbalAnglesToMotorAnglesTable::rowStartColIndices;

%include "gimbalAnglesToMotorAnglesTableAlgorithm.h"
%include "gimbalAnglesToMotorAnglesTable.h"

%include "msgPayloadDef/MotorAngleRefMsgF32Payload.h"
%include "msgPayloadDef/TwoAxisGimbalMsgF32Payload.h"
