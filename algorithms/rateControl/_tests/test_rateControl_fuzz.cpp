#include "rateControlTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

static auto Vec3(float lo, float hi) { return fuzztest::ArrayOf<3>(fuzztest::InRange(lo, hi)); }

inline void fuzzAdapterRateControl(const float ev1,
                                   const float ev2,
                                   const float sigma1,
                                   const float sigma2,
                                   const float sigma3,
                                   const float DerivativeGainP,
                                   const std::array<float, 3>& knownTorque_a,
                                   const std::array<float, 3>& omega_BR_a,
                                   const std::array<float, 3>& domega_RN_a) {
    const Eigen::Matrix3f spacecraftInertia = generateValidInertiaMatrix(ev1, ev2, sigma1, sigma2, sigma3);
    const Eigen::Vector3f knownTorquePntB_B(knownTorque_a[0], knownTorque_a[1], knownTorque_a[2]);
    const Eigen::Vector3f omega_BR_B(omega_BR_a[0], omega_BR_a[1], omega_BR_a[2]);
    const Eigen::Vector3f domega_RN_B(domega_RN_a[0], domega_RN_a[1], domega_RN_a[2]);
    regressionTestRateControl(spacecraftInertia, DerivativeGainP, knownTorquePntB_B, omega_BR_B, domega_RN_B);
}

FUZZ_TEST(rateControlFuzz, fuzzAdapterRateControl)
    .WithDomains(fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(0.0f, 1E6f),  // P must be >= 0
                 Vec3(-1E6f, 1E6f),              // known torque
                 Vec3(-1E6f, 1E6f),              // omega_BR
                 Vec3(-1E6f, 1E6f)               // domega_RN
    );
