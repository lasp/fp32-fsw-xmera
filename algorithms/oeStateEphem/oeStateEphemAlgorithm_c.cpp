#include "oeStateEphemAlgorithm_c.h"
#include "oeStateEphemAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>
#include <iterator>

// These entry points run on Ada tasks with tight stack budgets, so none of them may
// materialize the fit table or a config as a stack temporary. The config lives on the
// heap behind an opaque handle and is built arc by arc; the only transients here are
// single ~1 KB arcs. Validation follows the throw-in-C++, catch-in-the-shim layering:
// the config's mutators and validate() throw, and the bool-returning functions below
// are where those throws become status codes for the Ada side.

namespace {
// Copied field by field on purpose. The POD and C++ arcs agree on field order and
// anomalyFlag width, which makes them look castable -- but these copies are what keep
// ChebyshevFitArc's layout out of the Ada ABI, and std::array<double, N> matching
// double[N] is universal in practice rather than guaranteed. Do not replace either
// direction with a memcpy or a cast.
void arcFromC(ChebyshevFitArc& dst, const ChebyshevFitArc_c& src) {
    dst.numberChebCoefficients = src.numberChebCoefficients;
    dst.ephemerisTimeMiddle = src.ephemerisTimeMiddle;
    dst.ephemerisTimeRadius = src.ephemerisTimeRadius;
    dst.anomalyFlag = src.anomalyFlag;
    std::ranges::copy(src.radiusPeriapsisCoefficients, dst.radiusPeriapsisCoefficients.begin());
    std::ranges::copy(src.eccentricityCoefficients, dst.eccentricityCoefficients.begin());
    std::ranges::copy(src.inclinationCoefficients, dst.inclinationCoefficients.begin());
    std::ranges::copy(src.argPeriapsisCoefficients, dst.argPeriapsisCoefficients.begin());
    std::ranges::copy(src.raanCoefficients, dst.raanCoefficients.begin());
    std::ranges::copy(src.trueAnomalyCoefficients, dst.trueAnomalyCoefficients.begin());
}

void arcToC(ChebyshevFitArc_c& dst, const ChebyshevFitArc& src) {
    dst.numberChebCoefficients = src.numberChebCoefficients;
    dst.ephemerisTimeMiddle = src.ephemerisTimeMiddle;
    dst.ephemerisTimeRadius = src.ephemerisTimeRadius;
    dst.anomalyFlag = src.anomalyFlag;
    std::ranges::copy(src.radiusPeriapsisCoefficients, std::begin(dst.radiusPeriapsisCoefficients));
    std::ranges::copy(src.eccentricityCoefficients, std::begin(dst.eccentricityCoefficients));
    std::ranges::copy(src.inclinationCoefficients, std::begin(dst.inclinationCoefficients));
    std::ranges::copy(src.argPeriapsisCoefficients, std::begin(dst.argPeriapsisCoefficients));
    std::ranges::copy(src.raanCoefficients, std::begin(dst.raanCoefficients));
    std::ranges::copy(src.trueAnomalyCoefficients, std::begin(dst.trueAnomalyCoefficients));
}
}  // namespace

OEStateEphemConfigHandle* OEStateEphemConfig_create(void) {
    return fsw::createHandle<::OEStateEphemConfig, OEStateEphemConfigHandle>();
}

void OEStateEphemConfig_destroy(OEStateEphemConfigHandle* self) { fsw::deleteHandle<::OEStateEphemConfig>(self); }

void OEStateEphemConfig_reset(OEStateEphemConfigHandle* self) { fsw::fromHandle<::OEStateEphemConfig>(self)->reset(); }

bool OEStateEphemConfig_setScalars(OEStateEphemConfigHandle* self,
                                   const double centralBodyGravitationalParameter,
                                   const double ephemerisTimeJ2000,
                                   const double vehicleTimeOffset) {
    try {
        fsw::fromHandle<::OEStateEphemConfig>(self)->setScalars(
            centralBodyGravitationalParameter, ephemerisTimeJ2000, vehicleTimeOffset);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

bool OEStateEphemConfig_addArc(OEStateEphemConfigHandle* self, const ChebyshevFitArc_c* fitArc) {
    ChebyshevFitArc arc{};  // Single-arc transient, ~1 KB; the only stack cost of staging.
    arcFromC(arc, *fitArc);
    try {
        fsw::fromHandle<::OEStateEphemConfig>(self)->addArc(arc);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

bool OEStateEphemConfig_validate(OEStateEphemConfigHandle* self) {
    try {
        fsw::fromHandle<::OEStateEphemConfig>(self)->validate();
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

OEStateEphemAlgorithmHandle* OEStateEphemAlgorithm_create(OEStateEphemConfigHandle* config) {
    return fsw::createHandle<::OEStateEphemAlgorithm, OEStateEphemAlgorithmHandle>(
        *fsw::fromHandle<::OEStateEphemConfig>(config));
}

void OEStateEphemAlgorithm_destroy(OEStateEphemAlgorithmHandle* self) {
    fsw::deleteHandle<::OEStateEphemAlgorithm>(self);
}

void OEStateEphemAlgorithm_setConfig(OEStateEphemAlgorithmHandle* self, OEStateEphemConfigHandle* config) {
    fsw::fromHandle<::OEStateEphemAlgorithm>(self)->setConfig(*fsw::fromHandle<::OEStateEphemConfig>(config));
}

void OEStateEphemAlgorithm_getConfigScalars(OEStateEphemAlgorithmHandle* self,
                                            double* centralBodyGravitationalParameter,
                                            unsigned int* numberOfArcs,
                                            double* ephemerisTimeJ2000,
                                            double* vehicleTimeOffset) {
    const OEStateEphemConfig& config = fsw::fromHandle<::OEStateEphemAlgorithm>(self)->getConfig();
    *centralBodyGravitationalParameter = config.getCentralBodyGravitationalParameter();
    *numberOfArcs = config.getNumberOfArcs();
    *ephemerisTimeJ2000 = config.getEphemerisTimeJ2000();
    *vehicleTimeOffset = config.getVehicleTimeOffset();
}

void OEStateEphemAlgorithm_getConfigArc(OEStateEphemAlgorithmHandle* self,
                                        const unsigned int arcNumber,
                                        ChebyshevFitArc_c* fitArc) {
    const OEStateEphemConfig& config = fsw::fromHandle<::OEStateEphemAlgorithm>(self)->getConfig();
    arcToC(*fitArc, config.getFitCoefficients().at(arcNumber));
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

uint32_t OEStateEphemAlgorithm_getFitArcSizeBits(void) { return static_cast<uint32_t>(sizeof(ChebyshevFitArc_c) * 8U); }
