#include "ephemeridesRecenterAlgorithm_c.h"
#include "ephemeridesRecenterAlgorithm.h"

#include <Eigen/Core>
#include <array>
#include <cstddef>

EphemeridesRecenterAlgorithmHandle* EphemeridesRecenterAlgorithm_create(void) {
    return reinterpret_cast<EphemeridesRecenterAlgorithmHandle*>(new ::EphemeridesRecenterAlgorithm());
}

void EphemeridesRecenterAlgorithm_destroy(EphemeridesRecenterAlgorithmHandle* self) {
    delete reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self);
}

void EphemeridesRecenterAlgorithm_reset(EphemeridesRecenterAlgorithmHandle* self) {
    reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self)->reset();
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

void EphemeridesRecenterAlgorithm_setNewZeroBaseId(EphemeridesRecenterAlgorithmHandle* self, const int bodySpiceId) {
    reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self)->setNewZeroBaseId(bodySpiceId);
}

int EphemeridesRecenterAlgorithm_getNewZeroBase(const EphemeridesRecenterAlgorithmHandle* self) {
    return reinterpret_cast<const ::EphemeridesRecenterAlgorithm*>(self)->getNewZeroBase();
}

void EphemeridesRecenterAlgorithm_setPreviousCommonZeroBase(EphemeridesRecenterAlgorithmHandle* self,
                                                            const int bodySpiceId) {
    reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self)->setPreviousCommonZeroBase(bodySpiceId);
}

int EphemeridesRecenterAlgorithm_getPreviousCommonZeroBase(const EphemeridesRecenterAlgorithmHandle* self) {
    return reinterpret_cast<const ::EphemeridesRecenterAlgorithm*>(self)->getPreviousCommonZeroBase();
}

uint32_t EphemeridesRecenterAlgorithm_getNumberOfBodies(const EphemeridesRecenterAlgorithmHandle* self) {
    return static_cast<uint32_t>(reinterpret_cast<const ::EphemeridesRecenterAlgorithm*>(self)->getNumberOfBodies());
}

IntArray20_c EphemeridesRecenterAlgorithm_getAllIds(const EphemeridesRecenterAlgorithmHandle* self) {
    std::array<int, MAX_NUM_CHANGE_BODIES> ids =
        reinterpret_cast<const ::EphemeridesRecenterAlgorithm*>(self)->getAllIds();
    IntArray20_c out{};
    for (std::size_t i = 0U; i < MAX_NUM_CHANGE_BODIES; ++i) {
        out.id[i] = ids.at(i);
    }
    return out;
}

void EphemeridesRecenterAlgorithm_addBodyEphemerisToRecenter(EphemeridesRecenterAlgorithmHandle* self,
                                                             const BodyToRecenter* body) {
    reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self)->addBodyEphemerisToRecenter(*body);
}

void EphemeridesRecenterAlgorithm_clearAllBodies(EphemeridesRecenterAlgorithmHandle* self) {
    reinterpret_cast<::EphemeridesRecenterAlgorithm*>(self)->clearAllBodies();
}

uint32_t EphemeridesRecenterAlgorithm_findBodyIndex(const EphemeridesRecenterAlgorithmHandle* self,
                                                    const int bodySpiceId) {
    return static_cast<uint32_t>(
        reinterpret_cast<const ::EphemeridesRecenterAlgorithm*>(self)->findBodyIndex(bodySpiceId));
}

uint32_t EphemeridesRecenterAlgorithm_getMaxNumChangeBodies(void) { return MAX_NUM_CHANGE_BODIES; }
