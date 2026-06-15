#include "attTrackingErrorAlgorithm_c.h"
#include "attTrackingErrorAlgorithm.h"

#include <Eigen/Core>

AttTrackingErrorAlgorithmHandle* AttTrackingErrorAlgorithm_create(void) {
    // clang-format off
    return reinterpret_cast<AttTrackingErrorAlgorithmHandle*>(new ::AttTrackingErrorAlgorithm(AttTrackingErrorConfig::create()));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::AttTrackingErrorAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

AttGuidOutput_c AttTrackingErrorAlgorithm_update(AttTrackingErrorAlgorithmHandle* self,
                                                 AttNavInput_c navIn,
                                                 AttRefInput_c refIn) {
    // Convert C navigation input to Eigen-based struct
    AttNavInput nav{};
    nav.sigma_BN << navIn.sigma_BN.data[0], navIn.sigma_BN.data[1], navIn.sigma_BN.data[2];
    nav.omega_BN_B << navIn.omega_BN_B.data[0], navIn.omega_BN_B.data[1], navIn.omega_BN_B.data[2];

    // Convert C reference input to Eigen-based struct
    AttRefInput ref{};
    ref.sigma_RN << refIn.sigma_RN.data[0], refIn.sigma_RN.data[1], refIn.sigma_RN.data[2];
    ref.omega_RN_N << refIn.omega_RN_N.data[0], refIn.omega_RN_N.data[1], refIn.omega_RN_N.data[2];
    ref.domega_RN_N << refIn.domega_RN_N.data[0], refIn.domega_RN_N.data[1], refIn.domega_RN_N.data[2];

    // Call the algorithm
    // clang-format off
    const AttGuidOutput output = reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)->update(nav, ref);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    // Convert Eigen output back to C-compatible struct
    AttGuidOutput_c out{};
    out.sigma_BR = {output.sigma_BR[0], output.sigma_BR[1], output.sigma_BR[2]};
    out.omega_BR_B = {output.omega_BR_B[0], output.omega_BR_B[1], output.omega_BR_B[2]};
    out.omega_RN_B = {output.omega_RN_B[0], output.omega_RN_B[1], output.omega_RN_B[2]};
    out.domega_RN_B = {output.domega_RN_B[0], output.domega_RN_B[1], output.domega_RN_B[2]};

    return out;
}
