#include "ephemNavConverterAlgorithm_c.h"
#include "ephemNavConverterAlgorithm.h"

EphemNavConverterAlgorithm* EphemNavConverterAlgorithm_create(void) {
    return reinterpret_cast<EphemNavConverterAlgorithm*>(new ::EphemNavConverterAlgorithm());
}

void EphemNavConverterAlgorithm_destroy(EphemNavConverterAlgorithm* self) {
    delete reinterpret_cast<::EphemNavConverterAlgorithm*>(self);
}

OutputNavTransData EphemNavConverterAlgorithm_update(EphemNavConverterAlgorithm* self,
                                                     const InputEphemerisData* ephemerisInput) {
    return reinterpret_cast<::EphemNavConverterAlgorithm*>(self)->update(*ephemerisInput);
}
