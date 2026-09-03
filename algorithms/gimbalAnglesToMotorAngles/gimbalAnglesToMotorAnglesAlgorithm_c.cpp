#include "gimbalAnglesToMotorAnglesAlgorithm_c.h"
#include "gimbalAnglesToMotorAnglesAlgorithm.h"
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
GimbalAnglesToMotorAnglesConfig makeConfig(const MotorAngleRange_c* angleRange,
                                           const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                           const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                           const GimbalToMotorAngleTableLayout_c* tableLayout) {
    return GimbalAnglesToMotorAnglesConfig::create(StepperMotorAngleRange{angleRange->minAngle, angleRange->maxAngle},
                                                   toStdTable(gimbalToMotor1AngleTable),
                                                   toStdTable(gimbalToMotor2AngleTable),
                                                   toStdTableLayout(tableLayout));
}
}  // namespace

uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumTableRows(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; }

uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumTableCols(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_COLS; }

uint32_t GimbalAnglesToMotorAnglesAlgorithm_getNumTableElements(void) { return NUM_GIMBAL_TO_MOTOR_TABLE_ELEMENTS; }

bool GimbalAnglesToMotorAnglesAlgorithm_validateConfig(const MotorAngleRange_c* angleRange,
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

GimbalAnglesToMotorAnglesAlgorithmHandle* GimbalAnglesToMotorAnglesAlgorithm_create(
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableLayout_c* tableLayout) {
    return fsw::createHandle<::GimbalAnglesToMotorAnglesAlgorithm, GimbalAnglesToMotorAnglesAlgorithmHandle>(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout));
}

void GimbalAnglesToMotorAnglesAlgorithm_destroy(GimbalAnglesToMotorAnglesAlgorithmHandle* self) {
    fsw::deleteHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self);
}

void GimbalAnglesToMotorAnglesAlgorithm_setConfig(GimbalAnglesToMotorAnglesAlgorithmHandle* self,
                                                  const MotorAngleRange_c* angleRange,
                                                  const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                  const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                  const GimbalToMotorAngleTableLayout_c* tableLayout) {
    fsw::fromHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self)->setConfig(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout));
}

void GimbalAnglesToMotorAnglesAlgorithm_reInitialize(GimbalAnglesToMotorAnglesAlgorithmHandle* self) {
    fsw::fromHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self)->reInitialize();
}

GimbalAnglesToMotorAnglesOutput_c GimbalAnglesToMotorAnglesAlgorithm_update(
    GimbalAnglesToMotorAnglesAlgorithmHandle* self,
    const float gimbalAngle1,
    const float gimbalAngle2) {
    const GimbalAnglesToMotorAnglesOutput out =
        fsw::fromHandle<::GimbalAnglesToMotorAnglesAlgorithm>(self)->update(gimbalAngle1, gimbalAngle2);

    GimbalAnglesToMotorAnglesOutput_c result{};
    result.motorAngle1 = out.motorAngle1;
    result.motorAngle2 = out.motorAngle2;
    return result;
}
