#include "rwMotorTorqueAlgorithm_c.h"
#include "rwMotorTorqueAlgorithm.h"
#include "rwMotorTorqueTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
RwMotorTorqueArrayConfiguration arrayConfigurationFromC(const RwMotorTorqueArrayConfiguration_c& c) {
    RwMotorTorqueArrayConfiguration out{};
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(c.GsMatrix_B);
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        out.wheelAvailability[i] = fsw::toDeviceAvailability(c.wheelAvailability[i]);
    }
    return out;
}

// Reassemble the flattened C arguments into the C++ configuration structs. The flat argument list is the
// shape of the extern "C" boundary only; RwMotorTorqueConfig::create remains the single validation authority.
RwMotorTorqueConfig configFromC(const RwMotorTorqueControlAxes_c& desiredControlAxes_B,
                                const RwMotorTorqueArrayConfiguration_c& rwConfiguration,
                                const float omegaGain) {
    const std::array<bool, 3> axes{
        desiredControlAxes_B.axis[0] != 0, desiredControlAxes_B.axis[1] != 0, desiredControlAxes_B.axis[2] != 0};
    return RwMotorTorqueConfig::create(axes, arrayConfigurationFromC(rwConfiguration), omegaGain);
}
}  // namespace

uint32_t RwMotorTorqueAlgorithm_getMaxNumRw(void) { return kMaxNumRw; }

RwMotorTorqueAlgorithmHandle* RwMotorTorqueAlgorithm_create(const RwMotorTorqueControlAxes_c* desiredControlAxes_B,
                                                            const RwMotorTorqueArrayConfiguration_c* rwConfiguration,
                                                            const float omegaGain) {
    return fsw::createHandle<::RwMotorTorqueAlgorithm, RwMotorTorqueAlgorithmHandle>(
        configFromC(*desiredControlAxes_B, *rwConfiguration, omegaGain));
}

void RwMotorTorqueAlgorithm_destroy(RwMotorTorqueAlgorithmHandle* self) {
    fsw::deleteHandle<::RwMotorTorqueAlgorithm>(self);
}

void RwMotorTorqueAlgorithm_setConfig(RwMotorTorqueAlgorithmHandle* self,
                                      const RwMotorTorqueControlAxes_c* desiredControlAxes_B,
                                      const RwMotorTorqueArrayConfiguration_c* rwConfiguration,
                                      const float omegaGain) {
    fsw::fromHandle<::RwMotorTorqueAlgorithm>(self)->setConfig(
        configFromC(*desiredControlAxes_B, *rwConfiguration, omegaGain));
}

RwMotorTorqueOutput_c RwMotorTorqueAlgorithm_update(const RwMotorTorqueAlgorithmHandle* self,
                                                    const Vector3f_c Lr_B,
                                                    const RwSpeeds_c* rwSpeeds,
                                                    const RwSpeeds_c* rwDesiredSpeeds) {
    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds = cArrayToEigenVector(rwSpeeds->wheelSpeeds);
    speeds.rwDesiredSpeeds = cArrayToEigenVector(rwDesiredSpeeds->wheelSpeeds);

    const Eigen::Vector<float, kMaxNumRw> out =
        fsw::fromHandle<const ::RwMotorTorqueAlgorithm>(self)->update(cArrayToEigenVector3<float>(Lr_B.data), speeds);

    RwMotorTorqueOutput_c result{};
    eigenVectorToCArray(out, result.motorTorque);
    return result;
}
