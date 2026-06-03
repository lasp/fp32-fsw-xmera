#include "celestialTwoBodyPoint.h"

#include <stdexcept>

void CelestialTwoBodyPoint::reset(const uint64_t callTime) {
    this->secCelBodyIsLinked = this->secCelBodyInMsg.isLinked();

    // check if required input messages have been included
    if (!this->transNavInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.transNavInMsg was not linked.");
    }
    if (!this->celBodyInMsg.isLinked()) {
        throw std::invalid_argument("celestialTwoBodyPoint.celBodyInMsg was not linked.");
    }

    this->algorithm.reset(this->secCelBodyIsLinked);
}

/*! This method reads the input messages, computes the two-body celestial pointing attitude
 reference, and writes the attitude reference output message.
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void CelestialTwoBodyPoint::updateState(const uint64_t callTime) {
    const NavTransMsgF32Payload transNavIn = this->transNavInMsg();
    const EphemerisMsgF32Payload celBodyIn = this->celBodyInMsg();
    EphemerisMsgF32Payload secCelBodyIn{};
    if (this->secCelBodyIsLinked) {
        secCelBodyIn = this->secCelBodyInMsg();
    }

    AttRefMsgF32Payload attRefOut = this->algorithm.update(celBodyIn, secCelBodyIn, transNavIn);

    /*! - Write the output message */
    this->attRefOutMsg.write(&attRefOut, this->moduleID, callTime);
}

/**
 * @brief Set the singularity threshold
 * @param threshold singularity threshold
 */
void CelestialTwoBodyPoint::setSingularityThreshold(const float threshold) {
    this->algorithm.setSingularityThreshold(threshold);
}

/**
 * @brief Get the singularity threshold
 * @return float singularity threshold
 */
float CelestialTwoBodyPoint::getSingularityThreshold() const { return this->algorithm.getSingularityThreshold(); }

/**
 * @brief Set the rate threshold
 * @param rateThreshold rate threshold
 */
void CelestialTwoBodyPoint::setRateThreshold(const float rateThreshold) {
    this->algorithm.setRateThreshold(rateThreshold);
}

/**
 * @brief Get the rate threshold
 * @return float rate threshold
 */
float CelestialTwoBodyPoint::getRateThreshold() const { return this->algorithm.getRateThreshold(); }
