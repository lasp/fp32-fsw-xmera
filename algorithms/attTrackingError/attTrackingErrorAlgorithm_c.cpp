#include "attTrackingErrorAlgorithm_c.h"
#include "attTrackingErrorAlgorithm.h"
#include "attTrackingErrorTypes.h"
#include "utilities/fsw/eigenSupport.h"

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
    AttNavInput nav{};
    nav.sigma_BN = cArrayToEigenVector3<float>(navIn.sigma_BN.data);
    nav.omega_BN_B = cArrayToEigenVector3<float>(navIn.omega_BN_B.data);

    AttRefInput ref{};
    ref.sigma_RN = cArrayToEigenVector3<float>(refIn.sigma_RN.data);
    ref.omega_RN_N = cArrayToEigenVector3<float>(refIn.omega_RN_N.data);
    ref.domega_RN_N = cArrayToEigenVector3<float>(refIn.domega_RN_N.data);

    // clang-format off
    const AttGuidOutput output = reinterpret_cast<::AttTrackingErrorAlgorithm*>(self)->update(nav, ref);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    AttGuidOutput_c out{};
    eigenVectorToCArray(output.sigma_BR, out.sigma_BR.data);
    eigenVectorToCArray(output.omega_BR_B, out.omega_BR_B.data);
    eigenVectorToCArray(output.omega_RN_B, out.omega_RN_B.data);
    eigenVectorToCArray(output.domega_RN_B, out.domega_RN_B.data);

    return out;
}
