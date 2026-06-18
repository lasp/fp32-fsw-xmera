#include "rateControlTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>

static auto Vec3(double lo, double hi) { return fuzztest::ArrayOf<3>(fuzztest::InRange(lo, hi)); }

inline void fuzzAdapterRateControl(const float ev1,
                                   const float ev2,
                                   const float sigma1,
                                   const float sigma2,
                                   const float sigma3,
                                   const double DerivativeGainP,
                                   const Eigen::Vector3d knownTorque_a,
                                   const Eigen::Vector3d omega_BR_a,
                                   const Eigen::Vector3d domega_RN_a) {
    const Eigen::Matrix3f spacecraftInertia_f = generateValidInertiaMatrix(ev1, ev2, sigma1, sigma2, sigma3);

    const float derivativeGainP_f = static_cast<float>(DerivativeGainP);

    const Eigen::Vector3f knownTorquePntB_B_f = knownTorque_a.cast<float>();

    const Eigen::Vector3f omega_BR_B_f = omega_BR_a.cast<float>();

    const Eigen::Vector3f domega_RN_B_f = domega_RN_a.cast<float>();

    regressionTestRateControl(spacecraftInertia_f, derivativeGainP_f, knownTorquePntB_B_f, omega_BR_B_f, domega_RN_B_f);
}

FUZZ_TEST(rateControlFuzz, fuzzAdapterRateControl)
    .WithDomains(fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(0.0, 1.0E6),
                 xmera::fuzz::Vector3dInRange(-1.0E6, 1.0E6),
                 xmera::fuzz::Vector3dInRange(-1.0E6, 1.0E6),
                 xmera::fuzz::Vector3dInRange(-1.0E6, 1.0E6));
