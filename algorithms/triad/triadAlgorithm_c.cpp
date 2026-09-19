#include "triadAlgorithm_c.h"
#include "triadAlgorithm.h"
#include "triadTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

namespace {
TriadConfig configFromC(const Vector3f_c& sadaHat_B, const Vector3f_c& thrustReqHat_N, const N3Axis_c n3Axis) {
    return TriadConfig::create(cArrayToEigenVector3<float>(sadaHat_B.data),
                               cArrayToEigenVector3<float>(thrustReqHat_N.data),
                               static_cast<N3Axis>(n3Axis));
}
}  // namespace

bool TriadAlgorithm_validateConfig(const Vector3f_c* sadaHat_B,
                                   const Vector3f_c* thrustReqHat_N,
                                   const N3Axis_c n3Axis) {
    // Build the config through the same path create() uses: success means valid, a throw means
    // invalid. Sharing configFromC keeps the predicate from drifting from what create() accepts.
    try {
        (void)configFromC(*sadaHat_B, *thrustReqHat_N, n3Axis);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

TriadAlgorithmHandle* TriadAlgorithm_create(const Vector3f_c* sadaHat_B,
                                            const Vector3f_c* thrustReqHat_N,
                                            const N3Axis_c n3Axis) {
    return fsw::createHandle<::TriadAlgorithm, TriadAlgorithmHandle>(configFromC(*sadaHat_B, *thrustReqHat_N, n3Axis));
}

void TriadAlgorithm_destroy(TriadAlgorithmHandle* self) { fsw::deleteHandle<::TriadAlgorithm>(self); }

void TriadAlgorithm_setConfig(TriadAlgorithmHandle* self,
                              const Vector3f_c* sadaHat_B,
                              const Vector3f_c* thrustReqHat_N,
                              const N3Axis_c n3Axis) {
    fsw::fromHandle<::TriadAlgorithm>(self)->setConfig(configFromC(*sadaHat_B, *thrustReqHat_N, n3Axis));
}

Vector3f_c TriadAlgorithm_update(const TriadAlgorithmHandle* self,
                                 const Vector3f_c* rHat_SB_N,
                                 const Vector3f_c* thrustHat_B) {
    const Eigen::Vector3f sigma_RN = fsw::fromHandle<const ::TriadAlgorithm>(self)->update(
        cArrayToEigenVector3<float>(rHat_SB_N->data), cArrayToEigenVector3<float>(thrustHat_B->data));
    Vector3f_c result{};
    eigenVectorToCArray(sigma_RN, result.data);
    return result;
}
