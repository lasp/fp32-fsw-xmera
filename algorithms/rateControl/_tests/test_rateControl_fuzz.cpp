#include "rateControlTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

static auto Vec3(double lo, double hi) { return fuzztest::ArrayOf<3>(fuzztest::InRange(lo, hi)); }

inline void fuzzAdapterRateControl(const float ev1,
                                   const float ev2,
                                   const float sigma1,
                                   const float sigma2,
                                   const float sigma3,
                                   const double DerivativeGainP,
                                   const std::array<double, 3>& knownTorque_a,
                                   const std::array<double, 3>& omega_BR_a,
                                   const std::array<double, 3>& domega_RN_a) {
    const Eigen::Matrix3f spacecraftInertia_f = generateValidInertiaMatrix(ev1, ev2, sigma1, sigma2, sigma3);

    const float derivativeGainP_f = static_cast<float>(DerivativeGainP);

    const Eigen::Vector3f knownTorquePntB_B_f(static_cast<float>(knownTorque_a[0]),
                                              static_cast<float>(knownTorque_a[1]),
                                              static_cast<float>(knownTorque_a[2]));

    const Eigen::Vector3f omega_BR_B_f(
        static_cast<float>(omega_BR_a[0]), static_cast<float>(omega_BR_a[1]), static_cast<float>(omega_BR_a[2]));

    const Eigen::Vector3f domega_RN_B_f(
        static_cast<float>(domega_RN_a[0]), static_cast<float>(domega_RN_a[1]), static_cast<float>(domega_RN_a[2]));

    regressionTestRateControl(spacecraftInertia_f, derivativeGainP_f, knownTorquePntB_B_f, omega_BR_B_f, domega_RN_B_f);
}

FUZZ_TEST(rateControlFuzz, fuzzAdapterRateControl)
    .WithDomains(fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1.0F, 1e6F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(1e-6F, 1.0F),
                 fuzztest::InRange(0.0, 1.0E6),
                 Vec3(-1.0E6, 1.0E6),
                 Vec3(-1.0E6, 1.0E6),
                 Vec3(-1.0E6, 1.0E6));
