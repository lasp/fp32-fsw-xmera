#include "thrustAxisToMotorAnglesAlgorithm_c.h"

#include "thrustAxisToMotorAnglesAlgorithm.h"
#include "utilities/fsw/eigenSupport.h"

namespace {
/*! Convert a C-shared table POD into the algorithm's std::array table type. */
GimbalToMotorAngleTable toStdTable(const GimbalToMotorAngleTableData_c* src) {
    GimbalToMotorAngleTable out{};
    for (std::size_t index = 0; index < out.size(); ++index) {
        out[index] = src->data[index];
    }
    return out;
}

/*! Convert a C-shared table row index POD into the algorithm's std::array row layout type. */
GimbalToMotorAngleTableRowLayout toStdRowLayout(const GimbalToMotorAngleTableRowIndexData_c* src) {
    GimbalToMotorAngleTableRowLayout out{};
    for (std::size_t index = 0; index < out.size(); ++index) {
        out[index] = src->data[index];
    }
    return out;
}

/*! Build a validated Config from the C-shared configuration inputs. */
ThrustAxisToMotorAnglesConfig makeConfig(const float dcm_MB[3][3],
                                         const MotorAngleRange_c* angleRange,
                                         const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                         const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                         const GimbalToMotorAngleTableRowIndexData_c* rowStartStrideIndices,
                                         const GimbalToMotorAngleTableRowIndexData_c* rowStartColIndices) {
    const Eigen::Matrix3f dcm = cArrayToEigenMatrix3<float>(&dcm_MB[0][0]);
    return ThrustAxisToMotorAnglesConfig::create(dcm,
                                                 StepperMotorAngleRange{angleRange->minAngle, angleRange->maxAngle},
                                                 toStdTable(gimbalToMotor1AngleTable),
                                                 toStdTable(gimbalToMotor2AngleTable),
                                                 toStdRowLayout(rowStartStrideIndices),
                                                 toStdRowLayout(rowStartColIndices));
}
}  // namespace

ThrustAxisToMotorAnglesAlgorithmHandle* ThrustAxisToMotorAnglesAlgorithm_create(
    const float dcm_MB[3][3],
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
    const GimbalToMotorAngleTableRowIndexData_c* rowStartStrideIndices,
    const GimbalToMotorAngleTableRowIndexData_c* rowStartColIndices) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<ThrustAxisToMotorAnglesAlgorithmHandle*>(
        new ::ThrustAxisToMotorAnglesAlgorithm(makeConfig(dcm_MB,
                                                          angleRange,
                                                          gimbalToMotor1AngleTable,
                                                          gimbalToMotor2AngleTable,
                                                          rowStartStrideIndices,
                                                          rowStartColIndices)));
}

void ThrustAxisToMotorAnglesAlgorithm_destroy(ThrustAxisToMotorAnglesAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<::ThrustAxisToMotorAnglesAlgorithm*>(self);
}

void ThrustAxisToMotorAnglesAlgorithm_setConfig(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                const float dcm_MB[3][3],
                                                const MotorAngleRange_c* angleRange,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTableData_c* gimbalToMotor2AngleTable,
                                                const GimbalToMotorAngleTableRowIndexData_c* rowStartStrideIndices,
                                                const GimbalToMotorAngleTableRowIndexData_c* rowStartColIndices) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrustAxisToMotorAnglesAlgorithm*>(self)->setConfig(makeConfig(dcm_MB,
                                                                                      angleRange,
                                                                                      gimbalToMotor1AngleTable,
                                                                                      gimbalToMotor2AngleTable,
                                                                                      rowStartStrideIndices,
                                                                                      rowStartColIndices));
}

ThrustAxisToMotorAnglesOutput ThrustAxisToMotorAnglesAlgorithm_update(
    const ThrustAxisToMotorAnglesAlgorithmHandle* self,
    const float thrustHat_B[3]) {
    const Eigen::Vector3f thrustDir = cArrayToEigenVector3<float>(thrustHat_B);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::ThrustAxisToMotorAnglesAlgorithm*>(self)->update(thrustDir);
}
