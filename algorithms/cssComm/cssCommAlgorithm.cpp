#include "cssCommAlgorithm.h"
#include "utilities/fsw/chebyshevUtilities.h"

#include <algorithm>

CssCommAlgorithm::CssCommAlgorithm(const CssCommConfig& config) : cfg(config) { setConfig(config); }

void CssCommAlgorithm::setConfig(const CssCommConfig& config) { this->cfg = config; }

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return corrected CSS sensor values
 @param inputValues [-] Current measured CSS value for the constellation of CSS sensors
 */
std::array<double, kMaxNumCssSensors> CssCommAlgorithm::update(
    const std::array<double, kMaxNumCssSensors>& inputValues) const {
    std::array<double, kMaxNumCssSensors> outputValues{};

    for (uint32_t i = 0; i < this->cfg.getNumSensors(); ++i) {
        double const measuredValue = inputValues.at(i) / this->cfg.getMaxSensorValues().at(i); /* Scale Sensor Data */

        /* Calculate correction using Chebyshev polynomial */
        double const correction = calculateChebyValue(
            this->cfg.getChebyPolynomials(), static_cast<unsigned int>(kMaxNumChebyPolys), measuredValue);

        const double correctedValue = measuredValue + correction;

        /* A coarse sun sensor reports a cosine, so the corrected value cannot leave [0, 1]. A value that
           is not finite carries no measurement at all, so report no signal rather than pass it on. */
        outputValues.at(i) = fsw::is_finite(correctedValue) ? std::clamp(correctedValue, 0.0, 1.0) : 0.0;
    }

    return outputValues;
}
