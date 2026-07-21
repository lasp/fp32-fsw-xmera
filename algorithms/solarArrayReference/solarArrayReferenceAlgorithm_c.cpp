#include "solarArrayReferenceAlgorithm_c.h"
#include "solarArrayReferenceAlgorithm.h"
#include "solarArrayReferenceTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
SolarArrayReferenceConfig configFromC(const SolarArrayReferenceConfig_c& c) {
    return SolarArrayReferenceConfig::create(SolarArrayAxes{cArrayToEigenVector3<float>(c.driveAxis.data),
                                                            cArrayToEigenVector3<float>(c.surfaceNormal.data)},
                                             c.alignmentThreshold,
                                             c.trackingMode,
                                             c.specifiedArrayAngle,
                                             c.offsetAngle);
}
}  // namespace

SolarArrayReferenceAlgorithmHandle* SolarArrayReferenceAlgorithm_create(const SolarArrayReferenceConfig_c* config) {
    return fsw::createHandle<::SolarArrayReferenceAlgorithm, SolarArrayReferenceAlgorithmHandle>(configFromC(*config));
}

void SolarArrayReferenceAlgorithm_destroy(SolarArrayReferenceAlgorithmHandle* self) {
    fsw::deleteHandle<::SolarArrayReferenceAlgorithm>(self);
}

void SolarArrayReferenceAlgorithm_setConfig(SolarArrayReferenceAlgorithmHandle* self,
                                            const SolarArrayReferenceConfig_c* config) {
    fsw::fromHandle<::SolarArrayReferenceAlgorithm>(self)->setConfig(configFromC(*config));
}

float SolarArrayReferenceAlgorithm_update(const SolarArrayReferenceAlgorithmHandle* self,
                                          const Vector3f_c sigma_BN,
                                          const Vector3f_c sigma_RN,
                                          const Vector3f_c rHatIn_SB_B,
                                          const float theta) {
    return fsw::fromHandle<const ::SolarArrayReferenceAlgorithm>(self)->update(
        cArrayToEigenVector3<float>(sigma_BN.data),
        cArrayToEigenVector3<float>(sigma_RN.data),
        cArrayToEigenVector3<float>(rHatIn_SB_B.data),
        theta);
}
