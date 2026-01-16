#include "sunlineEphemAlgorithm_c.h"
#include "sunlineEphemAlgorithm.h"

SunlineEphemAlgorithm* SunlineEphemAlgorithm_create(void) {
    return reinterpret_cast<SunlineEphemAlgorithm*>(new ::SunlineEphemAlgorithm());
}

void SunlineEphemAlgorithm_destroy(SunlineEphemAlgorithm* self) {
    delete reinterpret_cast<::SunlineEphemAlgorithm*>(self);
}

NavAttMsgF32Payload SunlineEphemAlgorithm_updateState(const SunlineEphemAlgorithm* self,
                                                      const EphemerisMsgF32Payload* sunPos,
                                                      const NavTransMsgF32Payload* scPos,
                                                      const NavAttMsgF32Payload* scAtt) {
    return reinterpret_cast<const ::SunlineEphemAlgorithm*>(self)->updateState(*sunPos, *scPos, *scAtt);
}
