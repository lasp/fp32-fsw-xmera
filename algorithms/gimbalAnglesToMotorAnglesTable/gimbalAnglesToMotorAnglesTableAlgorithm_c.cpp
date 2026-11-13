#include "gimbalAnglesToMotorAnglesTableAlgorithm_c.h"
#include "gimbalAnglesToMotorAnglesTableAlgorithm.h"
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
GimbalAnglesToMotorAnglesTableConfig makeConfig(const MotorAngleRange_c* angleRange,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                const GimbalToMotorAngleTableLayout_c* tableLayout) {
    return GimbalAnglesToMotorAnglesTableConfig::create(
        StepperMotorAngleRange{angleRange->minAngle, angleRange->maxAngle},
        toStdTable(gimbalToMotor1AngleTable),
        toStdTable(gimbalToMotor2AngleTable),
        toStdTableLayout(tableLayout));
}
}  // namespace

uint32_t GimbalAnglesToMotorAnglesTableAlgorithm_getNumTableRows(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; }

uint32_t GimbalAnglesToMotorAnglesTableAlgorithm_getNumTableCols(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_COLS; }

uint32_t GimbalAnglesToMotorAnglesTableAlgorithm_getNumTableElements(void) {
    return NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS;
}

bool GimbalAnglesToMotorAnglesTableAlgorithm_validateConfig(
    const MotorAngleRange_c* angleRange,
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

GimbalAnglesToMotorAnglesTableAlgorithmHandle* GimbalAnglesToMotorAnglesTableAlgorithm_create(
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableLayout_c* tableLayout) {
    return fsw::createHandle<::GimbalAnglesToMotorAnglesTableAlgorithm, GimbalAnglesToMotorAnglesTableAlgorithmHandle>(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout));
}

void GimbalAnglesToMotorAnglesTableAlgorithm_destroy(GimbalAnglesToMotorAnglesTableAlgorithmHandle* self) {
    fsw::deleteHandle<::GimbalAnglesToMotorAnglesTableAlgorithm>(self);
}

void GimbalAnglesToMotorAnglesTableAlgorithm_setConfig(GimbalAnglesToMotorAnglesTableAlgorithmHandle* self,
                                                       const MotorAngleRange_c* angleRange,
                                                       const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                       const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                       const GimbalToMotorAngleTableLayout_c* tableLayout) {
    fsw::fromHandle<::GimbalAnglesToMotorAnglesTableAlgorithm>(self)->setConfig(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout));
}

void GimbalAnglesToMotorAnglesTableAlgorithm_reInitialize(GimbalAnglesToMotorAnglesTableAlgorithmHandle* self) {
    fsw::fromHandle<::GimbalAnglesToMotorAnglesTableAlgorithm>(self)->reInitialize();
}

GimbalAnglesToMotorAnglesTableOutput_c GimbalAnglesToMotorAnglesTableAlgorithm_update(
    GimbalAnglesToMotorAnglesTableAlgorithmHandle* self,
    const float gimbalAngle1,
    const float gimbalAngle2) {
    const GimbalAnglesToMotorAnglesTableOutput out =
        fsw::fromHandle<::GimbalAnglesToMotorAnglesTableAlgorithm>(self)->update(gimbalAngle1, gimbalAngle2);

    GimbalAnglesToMotorAnglesTableOutput_c result{};
    result.motorAngle1 = out.motorAngle1;
    result.motorAngle2 = out.motorAngle2;
    return result;
}
