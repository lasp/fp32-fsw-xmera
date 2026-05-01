/* SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2026 Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "torqueSchedulerAlgorithm_c.h"
#include "torqueSchedulerAlgorithm.h"
#include "torqueSchedulerTypes.h"

namespace {
TorqueSchedulerConfig configFromC(const TorqueSchedulerConfig_c& c) {
    return TorqueSchedulerConfig::create(static_cast<LockFlag>(c.lockFlag), c.tSwitch);
}
}  // namespace

TorqueSchedulerAlgorithm* TorqueSchedulerAlgorithm_create(const TorqueSchedulerConfig_c* config) {
    // clang-format off
    return reinterpret_cast<TorqueSchedulerAlgorithm*>(new ::TorqueSchedulerAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void TorqueSchedulerAlgorithm_destroy(TorqueSchedulerAlgorithm* self) {
    // clang-format off
    delete reinterpret_cast<::TorqueSchedulerAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void TorqueSchedulerAlgorithm_setConfig(TorqueSchedulerAlgorithm* self, const TorqueSchedulerConfig_c* config) {
    // clang-format off
    reinterpret_cast<::TorqueSchedulerAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

TorqueSchedulerOutput_c TorqueSchedulerAlgorithm_update(const TorqueSchedulerAlgorithm* self,
                                                        const float t,
                                                        const ArrayMotorTorqueMsgF32Payload* motorTorque1,
                                                        const ArrayMotorTorqueMsgF32Payload* motorTorque2) {
    // clang-format off
    const TorqueSchedulerOutput out = reinterpret_cast<const ::TorqueSchedulerAlgorithm*>(self)->update(t, *motorTorque1, *motorTorque2);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on

    TorqueSchedulerOutput_c result{};
    result.motorTorqueOut = out.motorTorqueOut;
    result.effectorLockOut = out.effectorLockOut;
    return result;
}
