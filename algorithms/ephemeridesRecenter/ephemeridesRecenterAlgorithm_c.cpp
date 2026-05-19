#include "ephemeridesRecenterAlgorithm_c.h"
#include "ephemeridesRecenterAlgorithm.h"

#include <array>
#include <cstddef>

namespace {
EphemeridesRecenterConfig configFromC(const EphemeridesRecenterConfig_c& c) {
    std::array<int, MAX_NUM_CHANGE_BODIES> bodyIds{};
    std::array<int, MAX_NUM_CHANGE_BODIES> originalCentralBodyIds{};
    for (std::size_t i = 0U; i < MAX_NUM_CHANGE_BODIES; ++i) {
        bodyIds.at(i) = c.bodyIds[i];
        originalCentralBodyIds.at(i) = c.originalCentralBodyIds[i];
    }
    return EphemeridesRecenterConfig::create(
        c.newCentralBodyId, c.previousCentralBodyId, bodyIds, originalCentralBodyIds, c.bodyCount);
}
}  // namespace

EphemeridesRecenterAlgorithmHandle* EphemeridesRecenterAlgorithm_create(const EphemeridesRecenterConfig_c* config) {
    return reinterpret_cast<EphemeridesRecenterAlgorithmHandle*>(
        new ::EphemeridesRecenterAlgorithm(configFromC(*config)));
}

void EphemeridesRecenterAlgorithm_destroy(EphemeridesRecenterAlgorithmHandle* self) {
    delete reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self);
}

void EphemeridesRecenterAlgorithm_setConfig(EphemeridesRecenterAlgorithmHandle* self,
                                            const EphemeridesRecenterConfig_c* config) {
    reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self)->setConfig(configFromC(*config));
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
        dst.position << src.position[0], src.position[1], src.position[2];
        dst.velocity << src.velocity[0], src.velocity[1], src.velocity[2];
    }

    std::array<BodyEphemerisPayload, MAX_NUM_CHANGE_BODIES> result =
        reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self)->updateState(input);

    BodyEphemerisPayloadArray20_c out{};
    for (std::size_t i = 0U; i < MAX_NUM_CHANGE_BODIES; ++i) {
        const BodyEphemerisPayload& src = result.at(i);
        BodyEphemerisPayload_c& dst = out.body[i];
        dst.bodySpiceId = src.bodySpiceId;
        dst.originalCentralBodyId = src.originalCentralBodyId;
        dst.isMoon = src.isMoon ? 1 : 0;
        dst.position[0] = src.position[0];
        dst.position[1] = src.position[1];
        dst.position[2] = src.position[2];
        dst.velocity[0] = src.velocity[0];
        dst.velocity[1] = src.velocity[1];
        dst.velocity[2] = src.velocity[2];
    }
    return out;
}

uint32_t EphemeridesRecenterAlgorithm_getMaxNumChangeBodies(void) { return MAX_NUM_CHANGE_BODIES; }
