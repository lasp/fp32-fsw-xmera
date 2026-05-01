/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "sunSafePointAlgorithm_c.h"
#include "sunSafePointAlgorithm.h"

#include <Eigen/Core>

SunSafePointAlgorithm* SunSafePointAlgorithm_create(void) {
    // clang-format off
    return reinterpret_cast<SunSafePointAlgorithm*>(new ::SunSafePointAlgorithm());  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SunSafePointAlgorithm_destroy(SunSafePointAlgorithm* self) {
    // clang-format off
    delete reinterpret_cast<::SunSafePointAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

SunSafePointOutput_c SunSafePointAlgorithm_update(const SunSafePointAlgorithm* self,
                                                  const Vector3f_c vehSunPntBdy,
                                                  const Vector3f_c omega_BN_B) {
    Eigen::Vector3f sun{};
    sun << vehSunPntBdy.data[0], vehSunPntBdy.data[1], vehSunPntBdy.data[2];

    Eigen::Vector3f omega{};
    omega << omega_BN_B.data[0], omega_BN_B.data[1], omega_BN_B.data[2];

    // clang-format off
    const SunSafePointOutput output = reinterpret_cast<const ::SunSafePointAlgorithm*>(self)->update(sun, omega);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    SunSafePointOutput_c out{};
    out.sigma_BR = {output.sigma_BR[0], output.sigma_BR[1], output.sigma_BR[2]};
    out.omega_BR_B = {output.omega_BR_B[0], output.omega_BR_B[1], output.omega_BR_B[2]};
    out.omega_RN_B = {output.omega_RN_B[0], output.omega_RN_B[1], output.omega_RN_B[2]};
    return out;
}

void SunSafePointAlgorithm_setSunAxisSpinRate(SunSafePointAlgorithm* self, const float rate) {
    // clang-format off
    reinterpret_cast<::SunSafePointAlgorithm*>(self)->setSunAxisSpinRate(rate);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

float SunSafePointAlgorithm_getSunAxisSpinRate(const SunSafePointAlgorithm* self) {
    // clang-format off
    return reinterpret_cast<const ::SunSafePointAlgorithm*>(self)->getSunAxisSpinRate();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SunSafePointAlgorithm_setOmega_RN_B(SunSafePointAlgorithm* self, const Vector3f_c omega) {
    Eigen::Vector3f eigenOmega{};
    eigenOmega << omega.data[0], omega.data[1], omega.data[2];
    // clang-format off
    reinterpret_cast<::SunSafePointAlgorithm*>(self)->setOmega_RN_B(eigenOmega);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

Vector3f_c SunSafePointAlgorithm_getOmega_RN_B(const SunSafePointAlgorithm* self) {
    // clang-format off
    const Eigen::Vector3f omega = reinterpret_cast<const ::SunSafePointAlgorithm*>(self)->getOmega_RN_B();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
    Vector3f_c out{};
    out.data[0] = omega[0];
    out.data[1] = omega[1];
    out.data[2] = omega[2];
    return out;
}

void SunSafePointAlgorithm_setSHatBdyCmd(SunSafePointAlgorithm* self, const Vector3f_c sHat) {
    Eigen::Vector3f eigenSHat{};
    eigenSHat << sHat.data[0], sHat.data[1], sHat.data[2];
    // clang-format off
    reinterpret_cast<::SunSafePointAlgorithm*>(self)->setSHatBdyCmd(eigenSHat);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

Vector3f_c SunSafePointAlgorithm_getSHatBdyCmd(const SunSafePointAlgorithm* self) {
    // clang-format off
    const Eigen::Vector3f sHat = reinterpret_cast<const ::SunSafePointAlgorithm*>(self)->getSHatBdyCmd();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
    Vector3f_c out{};
    out.data[0] = sHat[0];
    out.data[1] = sHat[1];
    out.data[2] = sHat[2];
    return out;
}
