#include "oeStateEphemAlgorithm_c.h"
#include "oeStateEphemAlgorithm.h"

#include <algorithm>

OEStateEphemAlgorithmHandle* OEStateEphemAlgorithm_create(void) {
    return reinterpret_cast<OEStateEphemAlgorithmHandle*>(new ::OEStateEphemAlgorithm());
}

void OEStateEphemAlgorithm_destroy(OEStateEphemAlgorithmHandle* self) {
    delete reinterpret_cast<::OEStateEphemAlgorithm*>(self);
}

CartesianState_c OEStateEphemAlgorithm_update(OEStateEphemAlgorithmHandle* self, const uint64_t callTime) {
    orbitalMotion::CartesianState result = reinterpret_cast<::OEStateEphemAlgorithm*>(self)->update(callTime);
    CartesianState_c out;
    out.position[0] = result.position[0];
    out.position[1] = result.position[1];
    out.position[2] = result.position[2];
    out.velocity[0] = result.velocity[0];
    out.velocity[1] = result.velocity[1];
    out.velocity[2] = result.velocity[2];
    return out;
}

void OEStateEphemAlgorithm_setCentralBodyGravitationalParameter(OEStateEphemAlgorithmHandle* self,
                                                                const double gravitationalParameter) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setCentralBodyGravitationalParameter(gravitationalParameter);
}

double OEStateEphemAlgorithm_getCentralBodyGravitationalParameter(const OEStateEphemAlgorithmHandle* self) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getCentralBodyGravitationalParameter();
}

void OEStateEphemAlgorithm_setNumberOfArcs(OEStateEphemAlgorithmHandle* self, const unsigned int arcs) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setNumberOfArcs(arcs);
}

unsigned int OEStateEphemAlgorithm_getNumberOfArcs(const OEStateEphemAlgorithmHandle* self) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getNumberOfArcs();
}

void OEStateEphemAlgorithm_setEphemerisTimeJ2000(OEStateEphemAlgorithmHandle* self, const double ephemerisJ2000) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setEphemerisTimeJ2000(ephemerisJ2000);
}

double OEStateEphemAlgorithm_getEphemerisTimeJ2000(const OEStateEphemAlgorithmHandle* self) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getEphemerisTimeJ2000();
}

void OEStateEphemAlgorithm_setVehicleTimeOffset(OEStateEphemAlgorithmHandle* self, const double timeOffset) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setVehicleTimeOffset(timeOffset);
}

double OEStateEphemAlgorithm_getVehicleTimeOffset(const OEStateEphemAlgorithmHandle* self) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getVehicleTimeOffset();
}

void OEStateEphemAlgorithm_setArcNumberOfCoefficients(OEStateEphemAlgorithmHandle* self,
                                                      const unsigned int arcNumber,
                                                      const unsigned int numberOfCoefficients) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcNumberOfCoefficients(arcNumber, numberOfCoefficients);
}

unsigned int OEStateEphemAlgorithm_getArcNumberOfCoefficients(const OEStateEphemAlgorithmHandle* self,
                                                              const unsigned int arcNumber) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcNumberOfCoefficients(arcNumber);
}

void OEStateEphemAlgorithm_setArcMiddleTime(OEStateEphemAlgorithmHandle* self,
                                            const unsigned int arcNumber,
                                            const double timeMiddle) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcMiddleTime(arcNumber, timeMiddle);
}

double OEStateEphemAlgorithm_getArcMiddleTime(const OEStateEphemAlgorithmHandle* self, const unsigned int arcNumber) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcMiddleTime(arcNumber);
}

void OEStateEphemAlgorithm_setArcRadiusTime(OEStateEphemAlgorithmHandle* self,
                                            const unsigned int arcNumber,
                                            const double timeRadius) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcRadiusTime(arcNumber, timeRadius);
}

double OEStateEphemAlgorithm_getArcRadiusTime(const OEStateEphemAlgorithmHandle* self, const unsigned int arcNumber) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcRadiusTime(arcNumber);
}

void OEStateEphemAlgorithm_setArcAnomalyFlag(OEStateEphemAlgorithmHandle* self,
                                             const unsigned int arcNumber,
                                             const AnomalyType anomalyFlag) {
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcAnomalyFlag(arcNumber, anomalyFlag);
}

AnomalyType OEStateEphemAlgorithm_getArcAnomalyFlag(const OEStateEphemAlgorithmHandle* self,
                                                    const unsigned int arcNumber) {
    return reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcAnomalyFlag(arcNumber);
}

void OEStateEphemAlgorithm_setArcRadiusPeriapsisCoefficients(OEStateEphemAlgorithmHandle* self,
                                                             const unsigned int arcNumber,
                                                             const OeCoefficients* coefficients) {
    std::array<double, kMaxOeCoeff> arr;
    std::copy(coefficients->data, coefficients->data + kMaxOeCoeff, arr.begin());
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcRadiusPeriapsisCoefficients(arcNumber, arr);
}

OeCoefficients OEStateEphemAlgorithm_getArcRadiusPeriapsisCoefficients(const OEStateEphemAlgorithmHandle* self,
                                                                       const unsigned int arcNumber) {
    auto arr = reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcRadiusPeriapsisCoefficients(arcNumber);
    OeCoefficients out;
    std::copy(arr.begin(), arr.end(), out.data);
    return out;
}

void OEStateEphemAlgorithm_setArcEccentricityCoefficients(OEStateEphemAlgorithmHandle* self,
                                                          const unsigned int arcNumber,
                                                          const OeCoefficients* coefficients) {
    std::array<double, kMaxOeCoeff> arr;
    std::copy(coefficients->data, coefficients->data + kMaxOeCoeff, arr.begin());
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcEccentricityCoefficients(arcNumber, arr);
}

OeCoefficients OEStateEphemAlgorithm_getArcEccentricityCoefficients(const OEStateEphemAlgorithmHandle* self,
                                                                    const unsigned int arcNumber) {
    auto arr = reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcEccentricityCoefficients(arcNumber);
    OeCoefficients out;
    std::copy(arr.begin(), arr.end(), out.data);
    return out;
}

void OEStateEphemAlgorithm_setArcInclinationCoefficients(OEStateEphemAlgorithmHandle* self,
                                                         const unsigned int arcNumber,
                                                         const OeCoefficients* coefficients) {
    std::array<double, kMaxOeCoeff> arr;
    std::copy(coefficients->data, coefficients->data + kMaxOeCoeff, arr.begin());
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcInclinationCoefficients(arcNumber, arr);
}

OeCoefficients OEStateEphemAlgorithm_getArcInclinationCoefficients(const OEStateEphemAlgorithmHandle* self,
                                                                   const unsigned int arcNumber) {
    auto arr = reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcInclinationCoefficients(arcNumber);
    OeCoefficients out;
    std::copy(arr.begin(), arr.end(), out.data);
    return out;
}

void OEStateEphemAlgorithm_setArcArgPeriapsisCoefficients(OEStateEphemAlgorithmHandle* self,
                                                          const unsigned int arcNumber,
                                                          const OeCoefficients* coefficients) {
    std::array<double, kMaxOeCoeff> arr;
    std::copy(coefficients->data, coefficients->data + kMaxOeCoeff, arr.begin());
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcArgPeriapsisCoefficients(arcNumber, arr);
}

OeCoefficients OEStateEphemAlgorithm_getArcArgPeriapsisCoefficients(const OEStateEphemAlgorithmHandle* self,
                                                                    const unsigned int arcNumber) {
    auto arr = reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcArgPeriapsisCoefficients(arcNumber);
    OeCoefficients out;
    std::copy(arr.begin(), arr.end(), out.data);
    return out;
}

void OEStateEphemAlgorithm_setArcRaanCoefficients(OEStateEphemAlgorithmHandle* self,
                                                  const unsigned int arcNumber,
                                                  const OeCoefficients* coefficients) {
    std::array<double, kMaxOeCoeff> arr;
    std::copy(coefficients->data, coefficients->data + kMaxOeCoeff, arr.begin());
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcRaanCoefficients(arcNumber, arr);
}

OeCoefficients OEStateEphemAlgorithm_getArcRaanCoefficients(const OEStateEphemAlgorithmHandle* self,
                                                            const unsigned int arcNumber) {
    auto arr = reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcRaanCoefficients(arcNumber);
    OeCoefficients out;
    std::copy(arr.begin(), arr.end(), out.data);
    return out;
}

void OEStateEphemAlgorithm_setArcTrueAnomalyCoefficients(OEStateEphemAlgorithmHandle* self,
                                                         const unsigned int arcNumber,
                                                         const OeCoefficients* coefficients) {
    std::array<double, kMaxOeCoeff> arr;
    std::copy(coefficients->data, coefficients->data + kMaxOeCoeff, arr.begin());
    reinterpret_cast<::OEStateEphemAlgorithm*>(self)->setArcTrueAnomalyCoefficients(arcNumber, arr);
}

OeCoefficients OEStateEphemAlgorithm_getArcTrueAnomalyCoefficients(const OEStateEphemAlgorithmHandle* self,
                                                                   const unsigned int arcNumber) {
    auto arr = reinterpret_cast<const ::OEStateEphemAlgorithm*>(self)->getArcTrueAnomalyCoefficients(arcNumber);
    OeCoefficients out;
    std::copy(arr.begin(), arr.end(), out.data);
    return out;
}

uint32_t OEStateEphemAlgorithm_getMaxOeCoeff(void) { return MAX_OE_COEFF; }

uint32_t OEStateEphemAlgorithm_getMaxOeRecords(void) { return MAX_OE_RECORDS; }
