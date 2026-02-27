/*
 MIT License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
 */

#include "oeStateEphem.h"
#include <architecture/utilities/eigenSupport.h>

/*!
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::reset(uint64_t callTime) {
    if (!this->clockCorrInMsg.isLinked()) {
        throw std::invalid_argument("OEStateEphem.clockCorrInMsg wasn't connected.");
    }
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void OEStateEphem::updateState(const uint64_t callTime) {
    EphemerisMsgF32Payload ephmerisMessageOutput{};

    if (this->clockCorrInMsg.isWritten()) {
        auto const timePayload = this->clockCorrInMsg();
        this->algorithm.setEphemerisTimeJ2000(timePayload.ephemerisTime);
        this->algorithm.setVehicleTime(timePayload.vehicleClockTime);
    }

    auto tmpOutputState = this->algorithm.update(callTime);

    ephmerisMessageOutput.timeTag = callTime * nanoToSeconds;
    eigenVectorToCArray(tmpOutputState.position, ephmerisMessageOutput.r_BdyZero_N);
    eigenVectorToCArray(tmpOutputState.velocity, ephmerisMessageOutput.v_BdyZero_N);
    this->stateFitOutMsg.write(&ephmerisMessageOutput, moduleID, callTime);
}

void OEStateEphem::setCentralBodyGravitationalParameter(const float mu) {
    this->algorithm.setCentralBodyGravitationalParameter(mu);
};

float OEStateEphem::getCentralBodyGravitationalParameter() const {
    return this->algorithm.getCentralBodyGravitationalParameter();
};

void OEStateEphem::setArcNumberOfCoefficients(const unsigned int arcNumber, const unsigned int numberOfCoefficients) {
    this->algorithm.setArcNumberOfCoefficients(arcNumber, numberOfCoefficients);
};

unsigned int OEStateEphem::getArcNumberOfCoefficients(const unsigned int arcNumber) const {
    return this->algorithm.getArcNumberOfCoefficients(arcNumber);
};

void OEStateEphem::setArcMiddleTime(const unsigned int arcNumber, const double timeMiddle) {
    this->algorithm.setArcMiddleTime(arcNumber, timeMiddle);
};

double OEStateEphem::getArcMiddleTime(const unsigned int arcNumber) const {
    return this->algorithm.getArcMiddleTime(arcNumber);
};

void OEStateEphem::setArcRadiusTime(const unsigned int arcNumber, const double timeRadius) {
    this->algorithm.setArcRadiusTime(arcNumber, timeRadius);
};

double OEStateEphem::getArcRadiusTime(const unsigned int arcNumber) const {
    return this->algorithm.getArcRadiusTime(arcNumber);
};

void OEStateEphem::setArcAnomalyFlag(const unsigned int arcNumber, const unsigned int anomalyFlag) {
    this->algorithm.setArcAnomalyFlag(arcNumber, anomalyFlag);
};

unsigned int OEStateEphem::getArcAnomalyFlag(unsigned int arcNumber) const {
    return this->algorithm.getArcAnomalyFlag(arcNumber);
};

void OEStateEphem::setArcRadiusPeriapsisCoefficients(
    const unsigned int arcNumber,
    const std::array<double, MAX_OE_COEFF> &radiusPeriapsisCoefficients) {
    this->algorithm.setArcRadiusPeriapsisCoefficients(arcNumber, radiusPeriapsisCoefficients);
};

std::array<double, MAX_OE_COEFF> OEStateEphem::getArcRadiusPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcRadiusPeriapsisCoefficients(arcNumber);
};

void OEStateEphem::setArcEccentricityCoefficients(const unsigned int arcNumber,
                                                  const std::array<float, MAX_OE_COEFF> &eccentricityCoefficients) {
    this->algorithm.setArcEccentricityCoefficients(arcNumber, eccentricityCoefficients);
};

std::array<float, MAX_OE_COEFF> OEStateEphem::getArcEccentricityCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcEccentricityCoefficients(arcNumber);
};

void OEStateEphem::setArcInclinationCoefficients(const unsigned int arcNumber,
                                                 const std::array<float, MAX_OE_COEFF> &inclinationCoefficients) {
    this->algorithm.setArcInclinationCoefficients(arcNumber, inclinationCoefficients);
};

std::array<float, MAX_OE_COEFF> OEStateEphem::getArcInclinationCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcInclinationCoefficients(arcNumber);
};

void OEStateEphem::setArcArgPeriapsisCoefficients(const unsigned int arcNumber,
                                                  const std::array<float, MAX_OE_COEFF> &argPeriapsisCoefficients) {
    this->algorithm.setArcArgPeriapsisCoefficients(arcNumber, argPeriapsisCoefficients);
};

std::array<float, MAX_OE_COEFF> OEStateEphem::getArcArgPeriapsisCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcArgPeriapsisCoefficients(arcNumber);
};

void OEStateEphem::setArcRaanCoefficients(const unsigned int arcNumber,
                                          const std::array<float, MAX_OE_COEFF> &raanCoefficients) {
    this->algorithm.setArcRaanCoefficients(arcNumber, raanCoefficients);
};

std::array<float, MAX_OE_COEFF> OEStateEphem::getArcRaanCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcRaanCoefficients(arcNumber);
};

void OEStateEphem::setArcTrueAnomalyCoefficients(const unsigned int arcNumber,
                                                 const std::array<float, MAX_OE_COEFF> &trueAnomalyCoefficients) {
    this->algorithm.setArcTrueAnomalyCoefficients(arcNumber, trueAnomalyCoefficients);
};

std::array<float, MAX_OE_COEFF> OEStateEphem::getArcTrueAnomalyCoefficients(const unsigned int arcNumber) {
    return this->algorithm.getArcTrueAnomalyCoefficients(arcNumber);
};
