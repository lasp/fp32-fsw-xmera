#include "ephemNavConverterAlgorithm_c.h"
#include "ephemNavConverterAlgorithm.h"

EphemNavConverterAlgorithm* EphemNavConverterAlgorithm_create(void) {
    return reinterpret_cast<EphemNavConverterAlgorithm*>(new ::EphemNavConverterAlgorithm());
}

void EphemNavConverterAlgorithm_destroy(EphemNavConverterAlgorithm* self) {
    delete reinterpret_cast<::EphemNavConverterAlgorithm*>(self);
}

NavTransMsgF32Payload EphemNavConverterAlgorithm_update(EphemNavConverterAlgorithm* self,
                                                        uint64_t callTime,
                                                        const EphemerisMsgF32Payload* ephemerisInMsg) {
    return reinterpret_cast<::EphemNavConverterAlgorithm*>(self)->update(callTime, *ephemerisInMsg);
}
