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
 * maximum value for each active sensor; a Chebyshev polynomial count in [1, kMaxNumChebyPolys]; and finite
 * polynomial coefficients. Construct via CssCommConfig::create(...).
 */
class CssCommConfig final {
   public:
    static CssCommConfig create(uint32_t numSensors,
                                const std::array<double, kMaxNumCssSensors>& maxSensorValues,
                                uint32_t chebyCount,
                                const std::array<double, kMaxNumChebyPolys>& chebyPolynomials) {
        if (!isValidNumSensors(numSensors)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: numSensors must be in [1, kMaxNumCssSensors]");
        }
        if (!isValidMaxSensorValues(maxSensorValues, numSensors)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: each active sensor's maxSensorValue must be finite and > 0");
        }
        if (!isValidChebyCount(chebyCount)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: chebyCount must be in [1, kMaxNumChebyPolys]");
        }
        if (!isValidChebyPolynomials(chebyPolynomials)) {
            FSW_THROW_INVALID_ARGUMENT("cssComm: chebyPolynomials must all be finite");
        }
        return {numSensors, maxSensorValues, chebyCount, chebyPolynomials};
    }

    static bool isValidNumSensors(uint32_t numSensors) { return numSensors >= 1U && numSensors <= kMaxNumCssSensors; }
    static bool isValidMaxSensorValues(const std::array<double, kMaxNumCssSensors>& maxSensorValues,
                                       uint32_t numSensors) {
        for (uint32_t i = 0U; i < numSensors && i < maxSensorValues.size(); ++i) {
            if (!fsw::is_finite(maxSensorValues.at(i)) || maxSensorValues.at(i) <= 0.0) {
                return false;
            }
        }
        return true;
    }
    static bool isValidChebyCount(uint32_t chebyCount) {
        return chebyCount >= 1U && chebyCount <= static_cast<uint32_t>(kMaxNumChebyPolys);
    }
    static bool isValidChebyPolynomials(const std::array<double, kMaxNumChebyPolys>& chebyPolynomials) {
        return std::ranges::all_of(chebyPolynomials, [](double coeff) { return fsw::is_finite(coeff); });
    }

    uint32_t getNumSensors() const { return numSensors; }
    const std::array<double, kMaxNumCssSensors>& getMaxSensorValues() const { return maxSensorValues; }
    uint32_t getChebyCount() const { return chebyCount; }
    const std::array<double, kMaxNumChebyPolys>& getChebyPolynomials() const { return chebyPolynomials; }

   private:
    // The scalar config fields have distinct meanings but mutually convertible types; construction is funneled
    // through the named create() factory, which makes the argument roles explicit at every call site.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    CssCommConfig(uint32_t numSensors,
                  const std::array<double, kMaxNumCssSensors>& maxSensorValues,
                  uint32_t chebyCount,
                  const std::array<double, kMaxNumChebyPolys>& chebyPolynomials)
        : numSensors(numSensors),
          maxSensorValues(maxSensorValues),
          chebyCount(chebyCount),
          chebyPolynomials(chebyPolynomials) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    uint32_t numSensors;
    std::array<double, kMaxNumCssSensors> maxSensorValues;
    uint32_t chebyCount;
    std::array<double, kMaxNumChebyPolys> chebyPolynomials;
};

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
class CssCommAlgorithm final {
   public:
    explicit CssCommAlgorithm(const CssCommConfig& config);
    void setConfig(const CssCommConfig& config);
    std::array<double, kMaxNumCssSensors> update(const std::array<double, kMaxNumCssSensors>& inputValues) const;

   private:
    CssCommConfig cfg;
};

#endif
