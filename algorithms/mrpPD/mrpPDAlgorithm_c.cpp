/* SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "mrpPDAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "mrpPDAlgorithm.h"
#include "mrpPDTypes.h"

#include <Eigen/Core>

namespace {
MrpPDConfig configFromC(const MrpPDConfig_c& c) {
    return MrpPDConfig::create(c.K,
                               c.P,
                               cArrayToEigenVector3<float>(c.knownTorquePntB_B.data),
                               cArrayToEigenMatrix3<float>(&c.spacecraftInertia.data[0][0]));
}
}  // namespace

MrpPDAlgorithm* MrpPDAlgorithm_create(const MrpPDConfig_c* config) {
    // clang-format off
    return reinterpret_cast<MrpPDAlgorithm*>(new ::MrpPDAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpPDAlgorithm_destroy(MrpPDAlgorithm* self) {
    // clang-format off
    delete reinterpret_cast<::MrpPDAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpPDAlgorithm_setConfig(MrpPDAlgorithm* self, const MrpPDConfig_c* config) {
    // clang-format off
    reinterpret_cast<::MrpPDAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

Vector3f_c MrpPDAlgorithm_update(MrpPDAlgorithm* self,
                                 const Vector3f_c* sigma_BR,
                                 const Vector3f_c* omega_BR_B,
                                 const Vector3f_c* domega_RN_B) {
    const Eigen::Vector3f sigma_BR_e = cArrayToEigenVector3<float>(sigma_BR->data);
    const Eigen::Vector3f omega_BR_B_e = cArrayToEigenVector3<float>(omega_BR_B->data);
    const Eigen::Vector3f domega_RN_B_e = cArrayToEigenVector3<float>(domega_RN_B->data);

    // clang-format off
    const Eigen::Vector3f Lr = reinterpret_cast<::MrpPDAlgorithm*>(self)->update(sigma_BR_e, omega_BR_B_e, domega_RN_B_e);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    Vector3f_c result{};
    eigenVectorToCArray(Lr, result.data);
    return result;
}
