#include "thrustAxisToMotorAnglesAlgorithm_c.h"

#include "architecture/utilities/eigenSupport.h"
#include "thrustAxisToMotorAnglesAlgorithm.h"

namespace {
/*! Convert a C-shared table POD into the algorithm's std::array table type. */
GimbalToMotorAngleTable toStdTable(const GimbalToMotorAngleTable_c* src) {
    GimbalToMotorAngleTable out{};
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        for (int col = 0; col < NUM_GIMBAL_TO_MOTOR_TABLE_COLS; ++col) {
            out[row][col] = src->data[row][col];
        }
    }
    return out;
}

/*! Build a validated Config from the C-shared configuration inputs. */
ThrustAxisToMotorAnglesConfig makeConfig(const float dcm_MB[3][3],
                                         const MotorAngleRange_c* angleRange,
                                         const GimbalToMotorAngleTable_c* gimbalToMotor1AngleTable,
                                         const GimbalToMotorAngleTable_c* gimbalToMotor2AngleTable) {
    const Eigen::Matrix3f dcm = cArrayToEigenMatrix3<float>(&dcm_MB[0][0]);
    return ThrustAxisToMotorAnglesConfig::create(dcm,
                                                 StepperMotorAngleRange{angleRange->minAngle, angleRange->maxAngle},
                                                 toStdTable(gimbalToMotor1AngleTable),
                                                 toStdTable(gimbalToMotor2AngleTable));
}
}  // namespace

ThrustAxisToMotorAnglesAlgorithmHandle* ThrustAxisToMotorAnglesAlgorithm_create(
    const float dcm_MB[3][3],
    const MotorAngleRange_c* angleRange,
    const GimbalToMotorAngleTable_c* gimbalToMotor1AngleTable,
    const GimbalToMotorAngleTable_c* gimbalToMotor2AngleTable) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<ThrustAxisToMotorAnglesAlgorithmHandle*>(new ::ThrustAxisToMotorAnglesAlgorithm(
        makeConfig(dcm_MB, angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable)));
}

void ThrustAxisToMotorAnglesAlgorithm_destroy(ThrustAxisToMotorAnglesAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<::ThrustAxisToMotorAnglesAlgorithm*>(self);
}

void ThrustAxisToMotorAnglesAlgorithm_setConfig(ThrustAxisToMotorAnglesAlgorithmHandle* self,
                                                const float dcm_MB[3][3],
                                                const MotorAngleRange_c* angleRange,
                                                const GimbalToMotorAngleTable_c* gimbalToMotor1AngleTable,
                                                const GimbalToMotorAngleTable_c* gimbalToMotor2AngleTable) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::ThrustAxisToMotorAnglesAlgorithm*>(self)->setConfig(
        makeConfig(dcm_MB, angleRange, gimbalToMotor1AngleTable, gimbalToMotor2AngleTable));
}

ThrustAxisToMotorAnglesOutput ThrustAxisToMotorAnglesAlgorithm_update(
    const ThrustAxisToMotorAnglesAlgorithmHandle* self,
    const float thrustHat_B[3]) {
    const Eigen::Vector3f thrustDir = cArrayToEigenVector3<float>(thrustHat_B);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::ThrustAxisToMotorAnglesAlgorithm*>(self)->update(thrustDir);
}
