#include "mrpRotationAlgorithm_c.h"
#include "architecture/utilities/eigenSupport.h"
#include "mrpRotationAlgorithm.h"
#include "mrpRotationTypes.h"

#include <Eigen/Core>

namespace {
MrpRotationConfig configFromC(const MrpRotationConfig_c& c) {
    return MrpRotationConfig::create(cArrayToEigenVector3<float>(c.initialSigmaRR0.data),
                                     cArrayToEigenVector3<float>(c.omegaRR0R.data),
                                     c.dynamicReferenceEnabled != 0);
}
}  // namespace

MrpRotationAlgorithmHandle* MrpRotationAlgorithm_create(const MrpRotationConfig_c* config) {
    // clang-format off
    return reinterpret_cast<MrpRotationAlgorithmHandle*>(new ::MrpRotationAlgorithm(configFromC(*config)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpRotationAlgorithm_destroy(MrpRotationAlgorithmHandle* self) {
    // clang-format off
    delete reinterpret_cast<::MrpRotationAlgorithm*>(self);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    // clang-format on
}

void MrpRotationAlgorithm_setConfig(MrpRotationAlgorithmHandle* self, const MrpRotationConfig_c* config) {
    // clang-format off
    reinterpret_cast<::MrpRotationAlgorithm*>(self)->setConfig(configFromC(*config));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

void MrpRotationAlgorithm_reset(MrpRotationAlgorithmHandle* self) {
    // clang-format off
    reinterpret_cast<::MrpRotationAlgorithm*>(self)->reset();  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}

AttRefMsgF32Payload MrpRotationAlgorithm_update(MrpRotationAlgorithmHandle* self,
                                                uint64_t callTime,
                                                const AttRefMsgF32Payload* inputRef,
                                                const AttStateMsgF32Payload* attStates) {
    // clang-format off
    return reinterpret_cast<::MrpRotationAlgorithm*>(self)->update(callTime, *inputRef, *attStates);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    // clang-format on
}
