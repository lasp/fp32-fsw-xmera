#ifndef F32XMERA_CSS_COMM_ALGORITHM_H
#define F32XMERA_CSS_COMM_ALGORITHM_H

#include "cssCommTypes.h"
#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

inline constexpr std::size_t kMaxNumChebyPolys = MAX_NUM_CHEBY_POLYS;

/*!
 * @brief Validated configuration for the CSS communication algorithm.
 *
 * An instance can only exist with: a sensor count in [1, kMaxNumCssSensors]; a finite, strictly positive
 * maximum sensor value; a Chebyshev polynomial count in [1, kMaxNumChebyPolys]; and finite polynomial
 * coefficients. Construct via CssCommConfig::create(...).
 */
class CssCommConfig final {
   public:
    static CssCommConfig create(uint32_t numSensors,
                                double maxSensorValue,
                                uint32_t chebyCount,
                                const std::array<double, kMaxNumChebyPolys>& chebyPolynomials) {
        if (!isValidNumSensors(numSensors)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: numSensors must be in [1, kMaxNumCssSensors]");
        }
        if (!isValidMaxSensorValue(maxSensorValue)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: maxSensorValue must be finite and > 0");
        }
        if (!isValidChebyCount(chebyCount)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: chebyCount must be in [1, kMaxNumChebyPolys]");
        }
        if (!isValidChebyPolynomials(chebyPolynomials)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: chebyPolynomials must all be finite");
        }
        return {numSensors, maxSensorValue, chebyCount, chebyPolynomials};
    }

    static bool isValidNumSensors(uint32_t numSensors) { return numSensors >= 1U && numSensors <= kMaxNumCssSensors; }
    static bool isValidMaxSensorValue(double maxSensorValue) {
        return fsw::is_finite(maxSensorValue) && maxSensorValue > 0.0;
    }
    static bool isValidChebyCount(uint32_t chebyCount) {
        return chebyCount >= 1U && chebyCount <= static_cast<uint32_t>(kMaxNumChebyPolys);
    }
    static bool isValidChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& chebyPolynomials) {
        return std::all_of(
            chebyPolynomials.begin(), chebyPolynomials.end(), [](double coeff) { return fsw::is_finite(coeff); });
    }

    uint32_t getNumSensors() const { return numSensors; }
    double getMaxSensorValue() const { return maxSensorValue; }
    uint32_t getChebyCount() const { return chebyCount; }
    const std::array<double, kMaxNumChebyPolys>& getChebyPolynomials() const { return chebyPolynomials; }

   private:
    CssCommConfig(uint32_t numSensors,
                  double maxSensorValue,
                  uint32_t chebyCount,
                  const std::array<double, kMaxNumChebyPolys>& chebyPolynomials)
        : numSensors(numSensors),
          maxSensorValue(maxSensorValue),
          chebyCount(chebyCount),
          chebyPolynomials(chebyPolynomials) {}

    uint32_t numSensors;
    double maxSensorValue;
    uint32_t chebyCount;
    std::array<double, kMaxNumChebyPolys> chebyPolynomials;
};

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CssCommAlgorithm final {
   public:
    std::array<double, kMaxNumCssSensors> update(const std::array<double, kMaxNumCssSensors>& inputValues) const;

    void setNumSensors(uint32_t numberOfSensors);
    uint32_t getNumSensors() const;
    void setMaxSensorValue(double maxValue);
    double getMaxSensorValue() const;
    void setChebyCount(uint32_t count);
    uint32_t getChebyCount() const;
    void setChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& polynomials);
    std::array<double, kMaxNumChebyPolys> getChebyPolynomials() const;

   private:
    uint32_t numSensors{};                                     //!< The number of sensors we are processing
    double maxSensorValue{};                                   //!< Scale factor to go from sensor values to cosine
    uint32_t chebyCount{};                                     //!< Count on the number of chebyshev polynomials we have
    std::array<double, kMaxNumChebyPolys> chebyPolynomials{};  //!< Chebyshev polynomials to fit output to cosine
};

#endif
