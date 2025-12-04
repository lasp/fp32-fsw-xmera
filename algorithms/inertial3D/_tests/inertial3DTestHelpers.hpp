#ifndef TEST_INERTIAL3D_H
#define TEST_INERTIAL3D_H

#include "architecture/utilities/eigenSupport.h"
#include "inertial3DAlgorithm.h"
#include "msgPayloadDef/AttRefMsgF32Payload.h"
#include <gtest/gtest.h>
#include <Eigen/Core>

// Reference computation for update
AttRefMsgF32Payload referenceUpdate(const Inertial3DAlgorithm& alg) {
    const Eigen::Vector3f sigma_RN = alg.getSigmaRN();

    AttRefMsgF32Payload out{};
    eigenVectorToCArray(sigma_RN, out.sigma_RN);
    return out;
}

void testInertial3D(std::vector<float> sigma) {
    Inertial3DAlgorithm alg;
    Eigen::Vector3f sigma_RN = Eigen::Map<Eigen::Vector3f>(sigma.data());
    alg.setSigmaRN(sigma_RN);

    AttRefMsgF32Payload out;
    EXPECT_NO_THROW(out = alg.update());
    AttRefMsgF32Payload ref;
    EXPECT_NO_THROW(ref = referenceUpdate(alg));

    for (int i = 0; i < 3; ++i) {
        // --- General tests ---

        // Reference correctness
        EXPECT_NEAR(out.sigma_RN[i], ref.sigma_RN[i], 1e-6);
        EXPECT_NEAR(out.omega_RN_N[i], ref.omega_RN_N[i], 1e-6);
        EXPECT_NEAR(out.domega_RN_N[i], ref.domega_RN_N[i], 1e-6);

        // Safety invariants
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
    }
}

#endif  // TEST_INERTIAL3D_H
