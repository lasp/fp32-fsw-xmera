#include "solarArrayReferenceAlgorithm_c.h"
#include "solarArrayReferenceAlgorithm.h"

#include <Eigen/Core>
#include <array>

SolarArrayReferenceAlgorithmHandle* SolarArrayReferenceAlgorithm_create(void) {
    // clang-format off
    return reinterpret_cast<SolarArrayReferenceAlgorithmHandle*>(new ::SolarArrayReferenceAlgorithm());  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SolarArrayReferenceAlgorithm_destroy(SolarArrayReferenceAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::SolarArrayReferenceAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

float SolarArrayReferenceAlgorithm_update(const SolarArrayReferenceAlgorithmHandle* self,
                                          const Vector3f_c sigma_BN,
                                          const Vector3f_c sigma_RN,
                                          const Vector3f_c rHatIn_SB_B,
                                          const float theta) {
    Eigen::Vector3f sigBN{};
    sigBN << sigma_BN.data[0], sigma_BN.data[1], sigma_BN.data[2];

    Eigen::Vector3f sigRN{};
    sigRN << sigma_RN.data[0], sigma_RN.data[1], sigma_RN.data[2];

    Eigen::Vector3f sun{};
    sun << rHatIn_SB_B.data[0], rHatIn_SB_B.data[1], rHatIn_SB_B.data[2];

    // clang-format off
    return reinterpret_cast<const ::SolarArrayReferenceAlgorithm*>(self)->update(sigBN, sigRN, sun, theta);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SolarArrayReferenceAlgorithm_setSolarArrayAxes_B(SolarArrayReferenceAlgorithmHandle* self,
                                                      const Vector3f_c driveAxis,
                                                      const Vector3f_c surfaceNormal) {
    Eigen::Vector3f drive{};
    drive << driveAxis.data[0], driveAxis.data[1], driveAxis.data[2];

    Eigen::Vector3f normal{};
    normal << surfaceNormal.data[0], surfaceNormal.data[1], surfaceNormal.data[2];

    // clang-format off
    reinterpret_cast<::SolarArrayReferenceAlgorithm*>(self)->setSolarArrayAxes_B(drive, normal);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

SolarArrayAxes_c SolarArrayReferenceAlgorithm_getSolarArrayAxes_B(const SolarArrayReferenceAlgorithmHandle* self) {
    // clang-format off
    const std::array<Eigen::Vector3f, 2> axes = reinterpret_cast<const ::SolarArrayReferenceAlgorithm*>(self)->getSolarArrayAxes_B();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
    SolarArrayAxes_c out{};
    out.driveAxis = {axes[0][0], axes[0][1], axes[0][2]};
    out.surfaceNormal = {axes[1][0], axes[1][1], axes[1][2]};
    return out;
}

void SolarArrayReferenceAlgorithm_setAlignmentThreshold(SolarArrayReferenceAlgorithmHandle* self,
                                                        const float threshold) {
    // clang-format off
    reinterpret_cast<::SolarArrayReferenceAlgorithm*>(self)->setAlignmentThreshold(threshold);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

float SolarArrayReferenceAlgorithm_getAlignmentThreshold(const SolarArrayReferenceAlgorithmHandle* self) {
    // clang-format off
    return reinterpret_cast<const ::SolarArrayReferenceAlgorithm*>(self)->getAlignmentThreshold();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SolarArrayReferenceAlgorithm_setTrackingMode(SolarArrayReferenceAlgorithmHandle* self, const TrackingMode mode) {
    // clang-format off
    reinterpret_cast<::SolarArrayReferenceAlgorithm*>(self)->setTrackingMode(mode);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

TrackingMode SolarArrayReferenceAlgorithm_getTrackingMode(const SolarArrayReferenceAlgorithmHandle* self) {
    // clang-format off
    return reinterpret_cast<const ::SolarArrayReferenceAlgorithm*>(self)->getTrackingMode();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SolarArrayReferenceAlgorithm_setSpecifiedArrayAngle(SolarArrayReferenceAlgorithmHandle* self, const float angle) {
    // clang-format off
    reinterpret_cast<::SolarArrayReferenceAlgorithm*>(self)->setSpecifiedArrayAngle(angle);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

float SolarArrayReferenceAlgorithm_getSpecifiedArrayAngle(const SolarArrayReferenceAlgorithmHandle* self) {
    // clang-format off
    return reinterpret_cast<const ::SolarArrayReferenceAlgorithm*>(self)->getSpecifiedArrayAngle();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void SolarArrayReferenceAlgorithm_setOffsetAngle(SolarArrayReferenceAlgorithmHandle* self, const float angle) {
    // clang-format off
    reinterpret_cast<::SolarArrayReferenceAlgorithm*>(self)->setOffsetAngle(angle);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

float SolarArrayReferenceAlgorithm_getOffsetAngle(const SolarArrayReferenceAlgorithmHandle* self) {
    // clang-format off
    return reinterpret_cast<const ::SolarArrayReferenceAlgorithm*>(self)->getOffsetAngle();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}
