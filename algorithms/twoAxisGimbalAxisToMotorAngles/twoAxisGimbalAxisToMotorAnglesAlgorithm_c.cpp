#include "twoAxisGimbalAxisToMotorAnglesAlgorithm_c.h"

#include "architecture/utilities/eigenSupport.h"
#include "twoAxisGimbalAxisToMotorAnglesAlgorithm.h"

namespace {
/*! Convert a C-shared table POD into the algorithm's std::array table type. */
GimbalMotorTable toStdTable(const GimbalMotorTable_c* src) {
    GimbalMotorTable out{};
    for (int row = 0; row < NUM_GIMBAL_TO_MOTOR_TABLE_ROWS; ++row) {
        for (int col = 0; col < NUM_GIMBAL_TO_MOTOR_TABLE_COLS; ++col) {
            out[row][col] = src->data[row][col];
        }
    }
    return out;
}

/*! Build a validated Config from the C-shared configuration inputs. */
TwoAxisGimbalAxisToMotorAnglesConfig makeConfig(const float dcm_MB[3][3],
                                                const GimbalMotorTable_c* gimbalToMotor1Data,
                                                const GimbalMotorTable_c* gimbalToMotor2Data) {
    const Eigen::Matrix3f dcm = cArrayToEigenMatrix3<float>(&dcm_MB[0][0]);
    return TwoAxisGimbalAxisToMotorAnglesConfig::create(
        dcm, toStdTable(gimbalToMotor1Data), toStdTable(gimbalToMotor2Data));
}
}  // namespace

TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* TwoAxisGimbalAxisToMotorAnglesAlgorithm_create(
    const float dcm_MB[3][3],
    const GimbalMotorTable_c* gimbalToMotor1Data,
    const GimbalMotorTable_c* gimbalToMotor2Data) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle*>(
        new ::TwoAxisGimbalAxisToMotorAnglesAlgorithm(makeConfig(dcm_MB, gimbalToMotor1Data, gimbalToMotor2Data)));
}

void TwoAxisGimbalAxisToMotorAnglesAlgorithm_destroy(TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<::TwoAxisGimbalAxisToMotorAnglesAlgorithm*>(self);
}

void TwoAxisGimbalAxisToMotorAnglesAlgorithm_setConfig(TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* self,
                                                       const float dcm_MB[3][3],
                                                       const GimbalMotorTable_c* gimbalToMotor1Data,
                                                       const GimbalMotorTable_c* gimbalToMotor2Data) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::TwoAxisGimbalAxisToMotorAnglesAlgorithm*>(self)->setConfig(
        makeConfig(dcm_MB, gimbalToMotor1Data, gimbalToMotor2Data));
}

TwoAxisGimbalAxisToMotorAnglesOutput TwoAxisGimbalAxisToMotorAnglesAlgorithm_update(
    const TwoAxisGimbalAxisToMotorAnglesAlgorithmHandle* self,
    const float thrustDirHat_B[3]) {
    const Eigen::Vector3f thrustDir = cArrayToEigenVector3<float>(thrustDirHat_B);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::TwoAxisGimbalAxisToMotorAnglesAlgorithm*>(self)->update(thrustDir);
}
