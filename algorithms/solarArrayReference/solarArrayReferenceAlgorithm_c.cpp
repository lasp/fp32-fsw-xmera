#include "solarArrayReferenceAlgorithm_c.h"
#include "solarArrayReferenceAlgorithm.h"
#include "solarArrayReferenceTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
SolarArrayReferenceConfig configFromC(const Vector3f_c& driveAxis,
                                      const Vector3f_c& surfaceNormal,
                                      const float alignmentThreshold,
                                      const TrackingMode trackingMode,
                                      const float specifiedArrayAngle,
                                      const float offsetAngle) {
    return SolarArrayReferenceConfig::create(
        SolarArrayAxes{cArrayToEigenVector3<float>(driveAxis.data), cArrayToEigenVector3<float>(surfaceNormal.data)},
        alignmentThreshold,
        trackingMode,
        specifiedArrayAngle,
        offsetAngle);
}
}  // namespace

SolarArrayReferenceAlgorithmHandle* SolarArrayReferenceAlgorithm_create(const Vector3f_c* driveAxis,
                                                                        const Vector3f_c* surfaceNormal,
                                                                        const float alignmentThreshold,
                                                                        const TrackingMode trackingMode,
                                                                        const float specifiedArrayAngle,
                                                                        const float offsetAngle) {
    return fsw::createHandle<::SolarArrayReferenceAlgorithm, SolarArrayReferenceAlgorithmHandle>(
        configFromC(*driveAxis, *surfaceNormal, alignmentThreshold, trackingMode, specifiedArrayAngle, offsetAngle));
}

void SolarArrayReferenceAlgorithm_destroy(SolarArrayReferenceAlgorithmHandle* self) {
    fsw::deleteHandle<::SolarArrayReferenceAlgorithm>(self);
}

void SolarArrayReferenceAlgorithm_setConfig(SolarArrayReferenceAlgorithmHandle* self,
                                            const Vector3f_c* driveAxis,
                                            const Vector3f_c* surfaceNormal,
                                            const float alignmentThreshold,
                                            const TrackingMode trackingMode,
                                            const float specifiedArrayAngle,
                                            const float offsetAngle) {
    fsw::fromHandle<::SolarArrayReferenceAlgorithm>(self)->setConfig(
        configFromC(*driveAxis, *surfaceNormal, alignmentThreshold, trackingMode, specifiedArrayAngle, offsetAngle));
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
