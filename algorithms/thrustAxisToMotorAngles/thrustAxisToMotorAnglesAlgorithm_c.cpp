#include "thrustAxisToMotorAnglesAlgorithm_c.h"

#include "thrustAxisToMotorAnglesAlgorithm.h"

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

/*! Build a validated Config from the C-shared configuration inputs. */
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

ThrustAxisToMotorAnglesAlgorithmHandle* ThrustAxisToMotorAnglesAlgorithm_create(
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableLayout_c* tableLayout) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<ThrustAxisToMotorAnglesAlgorithmHandle*>(new ::ThrustAxisToMotorAnglesAlgorithm(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout)));
}

void ThrustAxisToMotorAnglesAlgorithm_destroy(ThrustAxisToMotorAnglesAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<::ThrustAxisToMotorAnglesAlgorithm*>(self);
}

void ThrustAxisToMotorAnglesAlgorithm_setConfig(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                const MotorAngleRange_c* angleRange,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                const GimbalToMotorAngleTableLayout_c* tableLayout) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrustAxisToMotorAnglesAlgorithm*>(self)->setConfig(
        makeConfig(angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable, tableLayout));
}

ThrustAxisToMotorAnglesOutput_c ThrustAxisToMotorAnglesAlgorithm_update(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                                        const float gimbalAngle1,
                                                                        const float gimbalAngle2) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const ThrustAxisToMotorAnglesOutput out =
        reinterpret_cast<::ThrustAxisToMotorAnglesAlgorithm*>(self)->update(gimbalAngle1, gimbalAngle2);

    ThrustAxisToMotorAnglesOutput_c result{};
    result.motorAngle1 = out.motorAngle1;
    result.motorAngle2 = out.motorAngle2;
    return result;
}
