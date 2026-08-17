#include "thrustAxisToMotorAnglesAlgorithm_c.h"
#include "thrustAxisToMotorAnglesAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

namespace {
/*! Convert a C-shared table POD into the algorithm's std::array table type. */
GimbalToMotorAngleTable toStdTable(const GimbalToMotorAngleTableData_c* src) {
    GimbalToMotorAngleTable out{};
    for (std::size_t index = 0; index < out.size(); ++index) {
        out[index] = src->data[index];
    }
    return out;
}

/*! Convert a C-shared table row index POD into the algorithm's std::array row index type. */
std::array<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS> toStdRowIndices(const GimbalToMotorAngleTableRowIndexData_c* src) {
    std::array<int, NUM_GIMBAL_TO_MOTOR_TABLE_ROWS> out{};
    for (std::size_t index = 0; index < out.size(); ++index) {
        out[index] = src->data[index];
    }
    return out;
}

/*! Convert a C-shared table layout POD into the algorithm's table layout type. */
GimbalToMotorAngleTableLayout toStdTableLayout(const GimbalToMotorAngleTableLayout_c* src) {
    return GimbalToMotorAngleTableLayout{toStdRowIndices(&src->rowStartStrideIndices),
                                         toStdRowIndices(&src->rowStartColIndices),
                                         src->tipColIdxOffset,
                                         src->tiltRowIdxOffset,
                                         src->tableStepAngle};
}

/*! Build the validated C++ configuration from the flattened C parameters. */
ThrustAxisToMotorAnglesConfig makeConfig(const MotorAngleRange_c* angleRange,
                                         const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                         const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                         const GimbalToMotorAngleTableLayout_c* tableLayout) {
    return ThrustAxisToMotorAnglesConfig::create(StepperMotorAngleRange{angleRange->minAngle, angleRange->maxAngle},
                                                 toStdTable(gimbalToMotor1AngleTable),
                                                 toStdTable(gimbalToMotor2AngleTable),
                                                 toStdTableLayout(tableLayout));
}
}  // namespace

uint32_t ThrustAxisToMotorAnglesAlgorithm_getNumTableRows(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; }

uint32_t ThrustAxisToMotorAnglesAlgorithm_getNumTableCols(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_COLS; }

uint32_t ThrustAxisToMotorAnglesAlgorithm_getNumTableElements(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS; }

bool ThrustAxisToMotorAnglesAlgorithm_validateConfig(const MotorAngleRange_c* angleRange,
                                                     const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                     const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                     const GimbalToMotorAngleTableLayout_c* tableLayout) {
    try {
        (void)makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

ThrustAxisToMotorAnglesAlgorithmHandle* ThrustAxisToMotorAnglesAlgorithm_create(
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableLayout_c* tableLayout) {
    return fsw::createHandle<::ThrustAxisToMotorAnglesAlgorithm, ThrustAxisToMotorAnglesAlgorithmHandle>(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout));
}

void ThrustAxisToMotorAnglesAlgorithm_destroy(ThrustAxisToMotorAnglesAlgorithmHandle* self) {
    fsw::deleteHandle<::ThrustAxisToMotorAnglesAlgorithm>(self);
}

void ThrustAxisToMotorAnglesAlgorithm_setConfig(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                const MotorAngleRange_c* angleRange,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                const GimbalToMotorAngleTableLayout_c* tableLayout) {
    fsw::fromHandle<::ThrustAxisToMotorAnglesAlgorithm>(self)->setConfig(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout));
}

void ThrustAxisToMotorAnglesAlgorithm_reInitialize(ThrustAxisToMotorAnglesAlgorithmHandle* self) {
    fsw::fromHandle<::ThrustAxisToMotorAnglesAlgorithm>(self)->reInitialize();
}

ThrustAxisToMotorAnglesOutput_c ThrustAxisToMotorAnglesAlgorithm_update(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                                        const float gimbalAngle1,
                                                                        const float gimbalAngle2) {
    const ThrustAxisToMotorAnglesOutput out =
        fsw::fromHandle<::ThrustAxisToMotorAnglesAlgorithm>(self)->update(gimbalAngle1, gimbalAngle2);

    ThrustAxisToMotorAnglesOutput_c result{};
    result.motorAngle1 = out.motorAngle1;
    result.motorAngle2 = out.motorAngle2;
    return result;
}
