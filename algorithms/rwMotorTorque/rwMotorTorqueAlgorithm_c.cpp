#include "rwMotorTorqueAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "rwMotorTorqueAlgorithm.h"
#include "rwMotorTorqueTypes.h"

#include <Eigen/Core>

namespace {
RwMotorTorqueArrayConfiguration arrayConfigurationFromC(const RwMotorTorqueArrayConfiguration_c& c) {
    RwMotorTorqueArrayConfiguration out{};
    out.numRW = c.numRW;
    out.GsMatrix_B = cArrayToEigenMatrix<float, 3, kMaxNumRw>(c.GsMatrix_B);
    return out;
}

RwMotorTorqueAvailability availabilityFromC(const RwMotorTorqueAvailability_c& c) {
    RwMotorTorqueAvailability out{};
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        out.wheelAvailability[i] = c.wheelAvailability[i];
    }
    return out;
}

RwMotorTorqueConfig configFromC(const RwMotorTorqueConfig_c& c) {
    return RwMotorTorqueConfig::create(c2DArrayToEigenMatrix3(c.controlAxes_B.data),
                                       arrayConfigurationFromC(c.rwConfiguration),
                                       availabilityFromC(c.availability));
}
}  // namespace

uint32_t RwMotorTorqueAlgorithm_getMaxNumRw(void) { return RW_MOTOR_TORQUE_MAX_NUM_RW; }

RwMotorTorqueAlgorithmHandle* RwMotorTorqueAlgorithm_create(const RwMotorTorqueConfig_c* config) {
    return reinterpret_cast<RwMotorTorqueAlgorithmHandle*>(new ::RwMotorTorqueAlgorithm(configFromC(*config)));
}

void RwMotorTorqueAlgorithm_destroy(RwMotorTorqueAlgorithmHandle* self) {
    delete reinterpret_cast<::RwMotorTorqueAlgorithm*>(self);
}

void RwMotorTorqueAlgorithm_setConfig(RwMotorTorqueAlgorithmHandle* self, const RwMotorTorqueConfig_c* config) {
    reinterpret_cast<::RwMotorTorqueAlgorithm*>(self)->setConfig(configFromC(*config));
}

RwMotorTorqueOutput_c RwMotorTorqueAlgorithm_update(const RwMotorTorqueAlgorithmHandle* self, const Vector3f_c Lr_B) {
    const Eigen::Vector<float, kMaxNumRw> out =
        reinterpret_cast<const ::RwMotorTorqueAlgorithm*>(self)->update(cArrayToEigenVector3<float>(Lr_B.data));

    RwMotorTorqueOutput_c result{};
    eigenVectorToCArray(out, result.motorTorque);
    return result;
}
