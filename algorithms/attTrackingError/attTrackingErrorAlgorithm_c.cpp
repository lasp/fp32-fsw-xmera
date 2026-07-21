#include "attTrackingErrorAlgorithm_c.h"
#include "attTrackingErrorAlgorithm.h"
#include "attTrackingErrorTypes.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/opaqueHandle.h"

#include <Eigen/Core>

AttTrackingErrorAlgorithmHandle* AttTrackingErrorAlgorithm_create(void) {
    return fsw::createHandle<::AttTrackingErrorAlgorithm, AttTrackingErrorAlgorithmHandle>();
}

void AttTrackingErrorAlgorithm_destroy(AttTrackingErrorAlgorithmHandle* self) {
    fsw::deleteHandle<::AttTrackingErrorAlgorithm>(self);
}

AttGuidOutput_c AttTrackingErrorAlgorithm_update([[maybe_unused]] AttTrackingErrorAlgorithmHandle* self,
                                                 AttNavInput_c navIn,
                                                 AttRefInput_c refIn) {
    AttNavInput nav{};
    nav.sigma_BN = cArrayToEigenVector3<float>(navIn.sigma_BN.data);
    nav.omega_BN_B = cArrayToEigenVector3<float>(navIn.omega_BN_B.data);

    AttRefInput ref{};
    ref.sigma_RN = cArrayToEigenVector3<float>(refIn.sigma_RN.data);
    ref.omega_RN_N = cArrayToEigenVector3<float>(refIn.omega_RN_N.data);
    ref.domega_RN_N = cArrayToEigenVector3<float>(refIn.domega_RN_N.data);

    const AttGuidOutput output = ::AttTrackingErrorAlgorithm::update(nav, ref);

    AttGuidOutput_c out{};
    eigenVectorToCArray(output.sigma_BR, out.sigma_BR.data);
    eigenVectorToCArray(output.omega_BR_B, out.omega_BR_B.data);
    eigenVectorToCArray(output.omega_RN_B, out.omega_RN_B.data);
    eigenVectorToCArray(output.domega_RN_B, out.domega_RN_B.data);

    return out;
}
