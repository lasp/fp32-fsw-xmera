#include "oeStateEphem.h"

#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/timeConstants.h"
#include "utilities/xmera/xmeraLifecycleException.h"

#include <memory>
#include <stdexcept>

/*! Validate the input message link and build the validated algorithm configuration.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::reset(uint64_t callTime) {
    if (!this->clockCorrInMsg.isLinked()) {
        throw std::invalid_argument("OEStateEphem.clockCorrInMsg wasn't connected.");
    }

    const auto timePayload = this->clockCorrInMsg();
    auto config = OEStateEphemConfig::create(this->centralBodyGravitationalParameter,
                                             this->numberOfArcs,
                                             timePayload.ephemerisTime,
                                             timePayload.vehicleClockTime,
                                             this->fitCoefficients);
    this->algorithm = std::make_unique<OEStateEphemAlgorithm>(config);
}

OEStateEphemConfig OEStateEphem::toConfig() {
    const auto timePayload = this->clockCorrInMsg();
    return OEStateEphemConfig::create(this->centralBodyGravitationalParameter,
                                      this->numberOfArcs,
                                      timePayload.ephemerisTime,
                                      timePayload.vehicleClockTime,
                                      this->fitCoefficients);
}

void OEStateEphem::reconfigure() {
    if (!this->algorithm) {
        throw XmeraLifecycleException("OEStateEphem reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
}

/*! This method takes the current time and computes the state of the object using that time and the stored
    Chebyshev coefficients. If the time provided is outside the specified range, the position vectors rail
    high/low appropriately.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::updateState(const uint64_t callTime) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("OEStateEphem reset() has not been called.");
    }

    const auto tmpOutputState = this->algorithm->update(callTime);

    EphemerisMsgF32Payload ephmerisMessageOutput{};
    ephmerisMessageOutput.timeTag = callTime * kNano2Sec;
    ephmerisMessageOutput.spiceId = this->spiceBodyId;
    ephmerisMessageOutput.centralBodyId = this->centralBodyId;
    eigenVectorToCArray(tmpOutputState.position, ephmerisMessageOutput.r_BdyZero_N);
    eigenVectorToCArray(tmpOutputState.velocity, ephmerisMessageOutput.v_BdyZero_N);
    this->stateFitOutMsg.write(ephmerisMessageOutput, moduleID, callTime);
}

void OEStateEphem::setArcNumberOfCoefficients(const unsigned int arcNumber, const unsigned int numberOfCoefficients) {
    this->fitCoefficients.at(arcNumber).numberChebCoefficients = numberOfCoefficients;
}

unsigned int OEStateEphem::getArcNumberOfCoefficients(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).numberChebCoefficients;
}

void OEStateEphem::setArcMiddleTime(const unsigned int arcNumber, const double timeMiddle) {
    this->fitCoefficients.at(arcNumber).ephemerisTimeMiddle = timeMiddle;
}

double OEStateEphem::getArcMiddleTime(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).ephemerisTimeMiddle;
}

void OEStateEphem::setArcRadiusTime(const unsigned int arcNumber, const double timeRadius) {
    this->fitCoefficients.at(arcNumber).ephemerisTimeRadius = timeRadius;
}

double OEStateEphem::getArcRadiusTime(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).ephemerisTimeRadius;
}

void OEStateEphem::setArcAnomalyFlag(const unsigned int arcNumber, const AnomalyType& anomalyFlag) {
    this->fitCoefficients.at(arcNumber).anomalyFlag = anomalyFlag;
}

AnomalyType OEStateEphem::getArcAnomalyFlag(const unsigned int arcNumber) const {
    return this->fitCoefficients.at(arcNumber).anomalyFlag;
}

void OEStateEphem::setArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, kMaxOeCoeff>& radiusPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).radiusPeriapsisCoefficients = radiusPeriapsisCoefficients;
}

std::array<double, kMaxOeCoeff> OEStateEphem::getArcRadiusPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).radiusPeriapsisCoefficients;
}

void OEStateEphem::setArcEccentricityCoefficients(const unsigned int arcNumber,
                                                  const std::array<double, kMaxOeCoeff>& eccentricityCoefficients) {
    this->fitCoefficients.at(arcNumber).eccentricityCoefficients = eccentricityCoefficients;
}

std::array<double, kMaxOeCoeff> OEStateEphem::getArcEccentricityCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).eccentricityCoefficients;
}

void OEStateEphem::setArcInclinationCoefficients(const unsigned int arcNumber,
                                                 const std::array<double, kMaxOeCoeff>& inclinationCoefficients) {
    this->fitCoefficients.at(arcNumber).inclinationCoefficients = inclinationCoefficients;
}

std::array<double, kMaxOeCoeff> OEStateEphem::getArcInclinationCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).inclinationCoefficients;
}

void OEStateEphem::setArcArgPeriapsisCoefficients(const unsigned int arcNumber,
                                                  const std::array<double, kMaxOeCoeff>& argPeriapsisCoefficients) {
    this->fitCoefficients.at(arcNumber).argPeriapsisCoefficients = argPeriapsisCoefficients;
}

std::array<double, kMaxOeCoeff> OEStateEphem::getArcArgPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).argPeriapsisCoefficients;
}

void OEStateEphem::setArcRaanCoefficients(const unsigned int arcNumber,
                                          const std::array<double, kMaxOeCoeff>& raanCoefficients) {
    this->fitCoefficients.at(arcNumber).raanCoefficients = raanCoefficients;
}

std::array<double, kMaxOeCoeff> OEStateEphem::getArcRaanCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).raanCoefficients;
}

void OEStateEphem::setArcTrueAnomalyCoefficients(const unsigned int arcNumber,
                                                 const std::array<double, kMaxOeCoeff>& trueAnomalyCoefficients) {
    this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients = trueAnomalyCoefficients;
}

std::array<double, kMaxOeCoeff> OEStateEphem::getArcTrueAnomalyCoefficients(const unsigned int arcNumber) {
    return this->fitCoefficients.at(arcNumber).trueAnomalyCoefficients;
}
