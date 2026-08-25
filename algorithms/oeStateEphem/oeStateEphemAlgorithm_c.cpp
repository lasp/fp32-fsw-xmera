#include "oeStateEphemAlgorithm_c.h"
#include "oeStateEphemAlgorithm.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>
#include <array>
#include <cstddef>

namespace {
OEStateEphemConfig configFromC(const OEStateEphemConfig_c& config) {
    std::array<ChebyshevFitArc, kMaxOeRecords> arcs{};
    for (std::size_t i = 0; i < kMaxOeRecords; ++i) {
        const ChebyshevFitArc_c& src = config.fitCoefficients[i];
        ChebyshevFitArc& dst = arcs.at(i);
        // Copied field by field on purpose. The two structs now agree on field order and on
        // anomalyFlag's width, which makes them look castable -- but this copy is what keeps
        // ChebyshevFitArc's layout out of the Ada ABI, and std::array<double, N> matching
        // double[N] is universal in practice rather than guaranteed. Do not replace with a
        // memcpy or a reinterpret_cast.
        dst.numberChebCoefficients = src.numberChebCoefficients;
        dst.ephemerisTimeMiddle = src.ephemerisTimeMiddle;
        dst.ephemerisTimeRadius = src.ephemerisTimeRadius;
        dst.anomalyFlag = src.anomalyFlag;
        std::copy(std::begin(src.radiusPeriapsisCoefficients),
                  std::end(src.radiusPeriapsisCoefficients),
                  dst.radiusPeriapsisCoefficients.begin());
        std::copy(std::begin(src.eccentricityCoefficients),
                  std::end(src.eccentricityCoefficients),
                  dst.eccentricityCoefficients.begin());
        std::copy(std::begin(src.inclinationCoefficients),
                  std::end(src.inclinationCoefficients),
                  dst.inclinationCoefficients.begin());
        std::copy(std::begin(src.argPeriapsisCoefficients),
                  std::end(src.argPeriapsisCoefficients),
                  dst.argPeriapsisCoefficients.begin());
        std::copy(std::begin(src.raanCoefficients), std::end(src.raanCoefficients), dst.raanCoefficients.begin());
        std::copy(std::begin(src.trueAnomalyCoefficients),
                  std::end(src.trueAnomalyCoefficients),
                  dst.trueAnomalyCoefficients.begin());
    }
    return OEStateEphemConfig::create(config.centralBodyGravitationalParameter,
                                      config.numberOfArcs,
                                      config.ephemerisTimeJ2000,
                                      config.vehicleTimeOffset,
                                      arcs);
}
}  // namespace

OEStateEphemAlgorithmHandle* OEStateEphemAlgorithm_create(const OEStateEphemConfig_c* config) {
    return fsw::createHandle<::OEStateEphemAlgorithm, OEStateEphemAlgorithmHandle>(configFromC(*config));
}

void OEStateEphemAlgorithm_destroy(OEStateEphemAlgorithmHandle* self) {
    fsw::deleteHandle<::OEStateEphemAlgorithm>(self);
}

void OEStateEphemAlgorithm_setConfig(OEStateEphemAlgorithmHandle* self, const OEStateEphemConfig_c* config) {
    fsw::fromHandle<::OEStateEphemAlgorithm>(self)->setConfig(configFromC(*config));
}

CartesianState_c OEStateEphemAlgorithm_update(OEStateEphemAlgorithmHandle* self, const uint64_t callTime) {
    const orbitalMotion::CartesianState result = fsw::fromHandle<::OEStateEphemAlgorithm>(self)->update(callTime);
    CartesianState_c out;
    out.position[0] = result.position[0];
    out.position[1] = result.position[1];
    out.position[2] = result.position[2];
    out.velocity[0] = result.velocity[0];
    out.velocity[1] = result.velocity[1];
    out.velocity[2] = result.velocity[2];
    return out;
}

uint32_t OEStateEphemAlgorithm_getMaxOeCoeff(void) { return MAX_OE_COEFF; }

uint32_t OEStateEphemAlgorithm_getMaxOeRecords(void) { return MAX_OE_RECORDS; }
