// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "flybyPointAlgorithm_c.h"
#include "flybyPointAlgorithm.h"
#include <Eigen/Core>

namespace {
FlybyPointConfig configFromC(const FlybyPointConfig_c& c) {
    return FlybyPointConfig::create(c.timeBetweenFilterData,
                                    c.toleranceForCollinearity,
                                    c.signOfOrbitNormalFrameVector,
                                    c.maxRateThreshold,
                                    c.maxAccelerationThreshold,
                                    c.positionKnowledgeSigma);
}
}  // namespace

FlybyPointAlgorithmHandle* FlybyPointAlgorithm_create(const FlybyPointConfig_c* config) {
    // clang-format off
    return reinterpret_cast<FlybyPointAlgorithmHandle*>(new ::FlybyPointAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void FlybyPointAlgorithm_destroy(FlybyPointAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::FlybyPointAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void FlybyPointAlgorithm_reset(FlybyPointAlgorithmHandle* self) {
    // clang-format off
    reinterpret_cast<::FlybyPointAlgorithm*>(self)->reset();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void FlybyPointAlgorithm_setConfig(FlybyPointAlgorithmHandle* self, const FlybyPointConfig_c* config) {
    // clang-format off
    reinterpret_cast<::FlybyPointAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

FlybyPointOutput_c FlybyPointAlgorithm_updateState(FlybyPointAlgorithmHandle* self,
                                                   const uint64_t currentSimNanos,
                                                   const Vector3d_c r_BN_N,
                                                   const Vector3d_c v_BN_N) {
    Eigen::Vector3d r;
    r << r_BN_N.data[0], r_BN_N.data[1], r_BN_N.data[2];
    Eigen::Vector3d v;
    v << v_BN_N.data[0], v_BN_N.data[1], v_BN_N.data[2];
    // clang-format off
    const FlybyPointOutput out = reinterpret_cast<::FlybyPointAlgorithm*>(self)->updateState(currentSimNanos, r, v);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
    FlybyPointOutput_c result{};
    result.sigma_RN = {out.sigma_RN[0], out.sigma_RN[1], out.sigma_RN[2]};
    result.omega_RN_N = {out.omega_RN_N[0], out.omega_RN_N[1], out.omega_RN_N[2]};
    result.domega_RN_N = {out.domega_RN_N[0], out.domega_RN_N[1], out.domega_RN_N[2]};
    result.collinearityTrigger = out.collinearityTrigger;
    result.maxRateTrigger = out.maxRateTrigger;
    result.maxAccelerationTrigger = out.maxAccelerationTrigger;
    result.positionKnowledgeExceedTrigger = out.positionKnowledgeExceedTrigger;
    result.validOutput = out.validOutput;
    return result;
}
