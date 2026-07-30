#include "ephemeridesRecenterAlgorithm_c.h"
#include "ephemeridesRecenterAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

#include <array>
#include <cstddef>

namespace {
EphemeridesRecenterConfig configFromC(const int newCentralBodyId,
                                      const int previousCentralBodyId,
                                      const int bodyIds[MAX_NUM_CHANGE_BODIES],
                                      const int originalCentralBodyIds[MAX_NUM_CHANGE_BODIES],
                                      const uint32_t bodyCount) {
    std::array<int, MAX_NUM_CHANGE_BODIES> bodyIdsArr{};
    std::array<int, MAX_NUM_CHANGE_BODIES> originalCentralBodyIdsArr{};
    for (std::size_t i = 0U; i < MAX_NUM_CHANGE_BODIES; ++i) {
        bodyIdsArr.at(i) = bodyIds[i];
        originalCentralBodyIdsArr.at(i) = originalCentralBodyIds[i];
    }
    return EphemeridesRecenterConfig::create(
        newCentralBodyId, previousCentralBodyId, bodyIdsArr, originalCentralBodyIdsArr, bodyCount);
}
}  // namespace

EphemeridesRecenterAlgorithmHandle* EphemeridesRecenterAlgorithm_create(
    const int newCentralBodyId,
    const int previousCentralBodyId,
    int bodyIds[MAX_NUM_CHANGE_BODIES],
    int originalCentralBodyIds[MAX_NUM_CHANGE_BODIES],
    const uint32_t bodyCount) {
    return fsw::createHandle<::EphemeridesRecenterAlgorithm, EphemeridesRecenterAlgorithmHandle>(
        configFromC(newCentralBodyId, previousCentralBodyId, bodyIds, originalCentralBodyIds, bodyCount));
}

void EphemeridesRecenterAlgorithm_destroy(EphemeridesRecenterAlgorithmHandle* self) {
    fsw::deleteHandle<::EphemeridesRecenterAlgorithm>(self);
}

void EphemeridesRecenterAlgorithm_setConfig(EphemeridesRecenterAlgorithmHandle* self,
                                            const int newCentralBodyId,
                                            const int previousCentralBodyId,
                                            int bodyIds[MAX_NUM_CHANGE_BODIES],
                                            int originalCentralBodyIds[MAX_NUM_CHANGE_BODIES],
                                            const uint32_t bodyCount) {
    fsw::fromHandle<::EphemeridesRecenterAlgorithm>(self)->setConfig(
        configFromC(newCentralBodyId, previousCentralBodyId, bodyIds, originalCentralBodyIds, bodyCount));
}

BodyEphemerisPayloadArray20_c EphemeridesRecenterAlgorithm_updateState(EphemeridesRecenterAlgorithmHandle* self,
                                                                       const BodyEphemerisPayloadArray20_c* newBodies) {
    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> input{};
    for (std::size_t i = 0U; i < MAX_NUM_CHANGE_BODIES; ++i) {
        const BodyEphemerisPayload_c& src = newBodies->body[i];
        BodyEphemerisPayload& dst = input.at(i);
        dst.bodySpiceId = src.bodySpiceId;
        dst.originalCentralBodyId = src.originalCentralBodyId;
        dst.isMoon = (src.isMoon != 0);
        dst.position << src.input_r[0], src.input_r[1], src.input_r[2];
        dst.velocity << src.input_v[0], src.input_v[1], src.input_v[2];
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> result =
        fsw::fromHandle<::EphemeridesRecenterAlgorithm>(self)->updateState(input);

    BodyEphemerisPayloadArray20_c out{};
    for (std::size_t i = 0U; i < MAX_NUM_CHANGE_BODIES; ++i) {
        const BodyEphemerisPayload& src = result.at(i);
        BodyEphemerisPayload_c& dst = out.body[i];
        dst.bodySpiceId = src.bodySpiceId;
        dst.originalCentralBodyId = src.originalCentralBodyId;
        dst.isMoon = src.isMoon ? 1 : 0;
        dst.output_r[0] = src.position[0];
        dst.output_r[1] = src.position[1];
        dst.output_r[2] = src.position[2];
        dst.output_v[0] = src.velocity[0];
        dst.output_v[1] = src.velocity[1];
        dst.output_v[2] = src.velocity[2];
    }
    return out;
}

uint32_t EphemeridesRecenterAlgorithm_getMaxNumChangeBodies(void) { return MAX_NUM_CHANGE_BODIES; }
