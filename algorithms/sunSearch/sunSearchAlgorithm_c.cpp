/* MIT License
 *
 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "sunSearchAlgorithm_c.h"
#include "sunSearchAlgorithm.h"

SunSearchAlgorithm* SunSearchAlgorithm_create(void) {
    return reinterpret_cast<SunSearchAlgorithm*>(new ::SunSearchAlgorithm());
}

void SunSearchAlgorithm_destroy(SunSearchAlgorithm* self) { delete reinterpret_cast<::SunSearchAlgorithm*>(self); }

void SunSearchAlgorithm_reset(SunSearchAlgorithm* self,
                              const uint64_t currentSimNanos,
                              const PrincipleInertias* principleInertias) {
    reinterpret_cast<::SunSearchAlgorithm*>(self)->reset(currentSimNanos, *principleInertias);
}

AttGuidMsgF32Payload SunSearchAlgorithm_update(const SunSearchAlgorithm* self,
                                               const uint64_t currentSimNanos,
                                               const NavAttMsgF32Payload* navAttIn) {
    return reinterpret_cast<const ::SunSearchAlgorithm*>(self)->update(currentSimNanos, *navAttIn);
}

void SunSearchAlgorithm_setSlewProperties(SunSearchAlgorithm* self, const SlewProperties* slewPropertiesInput) {
    reinterpret_cast<::SunSearchAlgorithm*>(self)->setSlewProperties(*slewPropertiesInput);
}

void SunSearchAlgorithm_modifySlewProperties(SunSearchAlgorithm* self,
                                             const SlewProperties* slewPropertiesInput,
                                             const uint32_t index) {
    reinterpret_cast<::SunSearchAlgorithm*>(self)->modifySlewProperties(*slewPropertiesInput, index);
}

SlewProperties SunSearchAlgorithm_getSlewProperties(const SunSearchAlgorithm* self, const uint32_t index) {
    return reinterpret_cast<const ::SunSearchAlgorithm*>(self)->getSlewProperties(index);
}

uint32_t SunSearchAlgorithm_getNumSlews(void) { return NUM_SLEWS; }
