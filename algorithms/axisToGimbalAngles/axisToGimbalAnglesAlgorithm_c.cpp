#include "axisToGimbalAnglesAlgorithm_c.h"
#include "axisToGimbalAnglesAlgorithm.h"
#include "axisToGimbalAnglesTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
/*! Build the validated C++ configuration from the flattened C parameters. */
AxisToGimbalAnglesConfig makeConfig(const Vector3f_c& sigma_MB) {
    return AxisToGimbalAnglesConfig::create(cArrayToEigenVector3<float>(sigma_MB.data));
}
}  // namespace

bool AxisToGimbalAnglesAlgorithm_validateConfig(const Vector3f_c* sigma_MB) {
    try {
        (void)makeConfig(*sigma_MB);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

AxisToGimbalAnglesAlgorithmHandle* AxisToGimbalAnglesAlgorithm_create(const Vector3f_c* sigma_MB) {
    return fsw::createHandle<::AxisToGimbalAnglesAlgorithm, AxisToGimbalAnglesAlgorithmHandle>(makeConfig(*sigma_MB));
}

void AxisToGimbalAnglesAlgorithm_destroy(AxisToGimbalAnglesAlgorithmHandle* self) {
    fsw::deleteHandle<::AxisToGimbalAnglesAlgorithm>(self);
}

void AxisToGimbalAnglesAlgorithm_setConfig(AxisToGimbalAnglesAlgorithmHandle* self, const Vector3f_c* sigma_MB) {
    fsw::fromHandle<::AxisToGimbalAnglesAlgorithm>(self)->setConfig(makeConfig(*sigma_MB));
}

AxisToGimbalAnglesOutput_c AxisToGimbalAnglesAlgorithm_update(const AxisToGimbalAnglesAlgorithmHandle* self,
                                                              const Vector3f_c* thrustHat_B) {
    const AxisToGimbalAnglesOutput out = fsw::fromHandle<const ::AxisToGimbalAnglesAlgorithm>(self)->update(
        cArrayToEigenVector3<float>(thrustHat_B->data));

    AxisToGimbalAnglesOutput_c result{};
    result.gimbalAngle1 = out.gimbalAngle1;
    result.gimbalAngle2 = out.gimbalAngle2;
    return result;
}
