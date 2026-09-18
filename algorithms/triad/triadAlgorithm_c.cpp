#include "triadAlgorithm_c.h"
#include "triadAlgorithm.h"
#include "triadTypes.h"
#include "utilities/fsw/eigenSupport.h"

#include <Eigen/Core>

namespace {
TriadConfig configFromC(const Vector3f_c& sadaHat_B, const Vector3f_c& thrustReqHat_N, const N3Axis_c n3Axis) {
    return TriadConfig::create(cArrayToEigenVector3<float>(sadaHat_B.data),
                               cArrayToEigenVector3<float>(thrustReqHat_N.data),
                               static_cast<N3Axis>(n3Axis));
}
}  // namespace

TriadAlgorithmHandle* TriadAlgorithm_create(const Vector3f_c* sadaHat_B,
                                            const Vector3f_c* thrustReqHat_N,
                                            const N3Axis_c n3Axis) {
    // clang-format off
    return reinterpret_cast<TriadAlgorithmHandle*>(new ::TriadAlgorithm(configFromC(*sadaHat_B, *thrustReqHat_N, n3Axis)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void TriadAlgorithm_destroy(TriadAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::TriadAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void TriadAlgorithm_setConfig(TriadAlgorithmHandle* self,
                              const Vector3f_c* sadaHat_B,
                              const Vector3f_c* thrustReqHat_N,
                              const N3Axis_c n3Axis) {
    // clang-format off
    reinterpret_cast<::TriadAlgorithm*>(self)->setConfig(configFromC(*sadaHat_B, *thrustReqHat_N, n3Axis));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

Vector3f_c TriadAlgorithm_update(TriadAlgorithmHandle* self,
                                 const Vector3f_c* rHat_SB_N,
                                 const Vector3f_c* thrustHat_B) {
    // clang-format off
    const Eigen::Vector3f sigma_RN = reinterpret_cast<::TriadAlgorithm*>(self)->update(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        cArrayToEigenVector3<float>(rHat_SB_N->data),
        cArrayToEigenVector3<float>(thrustHat_B->data));
    // clang-format on
    Vector3f_c result{};
    eigenVectorToCArray(sigma_RN, result.data);
    return result;
}
