#include "sunSearchAlgorithm_c.h"
#include "sunSearchAlgorithm.h"

SunSearchAlgorithmHandle* SunSearchAlgorithm_create(void) {
    return reinterpret_cast<SunSearchAlgorithmHandle*>(new ::SunSearchAlgorithm());
}

void SunSearchAlgorithm_destroy(SunSearchAlgorithmHandle* self) {
    delete reinterpret_cast<::SunSearchAlgorithm*>(self);
}

void SunSearchAlgorithm_reset(SunSearchAlgorithmHandle* self,
                              const uint64_t currentSimNanos,
                              const PrincipleInertias* principleInertias) {
    reinterpret_cast<::SunSearchAlgorithm*>(self)->reset(currentSimNanos, *principleInertias);
}

AttGuidMsgF32Payload SunSearchAlgorithm_update(const SunSearchAlgorithmHandle* self,
                                               const uint64_t currentSimNanos,
                                               const NavAttMsgF32Payload* navAttIn) {
    return reinterpret_cast<const ::SunSearchAlgorithm*>(self)->update(currentSimNanos, *navAttIn);
}

void SunSearchAlgorithm_setSlewProperties(SunSearchAlgorithmHandle* self, const SlewProperties* slewPropertiesInput) {
    reinterpret_cast<::SunSearchAlgorithm*>(self)->setSlewProperties(*slewPropertiesInput);
}

void SunSearchAlgorithm_modifySlewProperties(SunSearchAlgorithmHandle* self,
                                             const SlewProperties* slewPropertiesInput,
                                             const uint32_t index) {
    reinterpret_cast<::SunSearchAlgorithm*>(self)->modifySlewProperties(*slewPropertiesInput, index);
}

SlewProperties SunSearchAlgorithm_getSlewProperties(const SunSearchAlgorithmHandle* self, const uint32_t index) {
    return reinterpret_cast<const ::SunSearchAlgorithm*>(self)->getSlewProperties(index);
}

uint32_t SunSearchAlgorithm_getNumSlews(void) { return NUM_SLEWS; }
