#include "cssCommAlgorithm_c.h"
#include "cssCommAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

#include <algorithm>
#include <array>

namespace {
CssCommConfig configFromC(const uint32_t numSensors,
                          double maxSensorValues[MAX_NUM_CSS_SENSORS],
                          double chebyPolynomials[MAX_NUM_CHEBY_POLYS]) {
    std::array<double, kMaxNumChebyPolys> cheby{};
    std::copy(chebyPolynomials, chebyPolynomials + kMaxNumChebyPolys, cheby.begin());
    std::array<double, kMaxNumCssSensors> maxValues{};
    std::copy(maxSensorValues, maxSensorValues + kMaxNumCssSensors, maxValues.begin());
    return CssCommConfig::create(numSensors, maxValues, cheby);
}
}  // namespace

bool CssCommAlgorithm_validateConfig(const uint32_t numSensors,
                                     double maxSensorValues[MAX_NUM_CSS_SENSORS],
                                     double chebyPolynomials[MAX_NUM_CHEBY_POLYS]) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(numSensors, maxSensorValues, chebyPolynomials);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

CssCommAlgorithmHandle* CssCommAlgorithm_create(const uint32_t numSensors,
                                                double maxSensorValues[MAX_NUM_CSS_SENSORS],
                                                double chebyPolynomials[MAX_NUM_CHEBY_POLYS]) {
    return fsw::createHandle<::CssCommAlgorithm, CssCommAlgorithmHandle>(
        configFromC(numSensors, maxSensorValues, chebyPolynomials));
}

void CssCommAlgorithm_destroy(CssCommAlgorithmHandle* self) { fsw::deleteHandle<::CssCommAlgorithm>(self); }

void CssCommAlgorithm_setConfig(CssCommAlgorithmHandle* self,
                                const uint32_t numSensors,
                                double maxSensorValues[MAX_NUM_CSS_SENSORS],
                                double chebyPolynomials[MAX_NUM_CHEBY_POLYS]) {
    fsw::fromHandle<::CssCommAlgorithm>(self)->setConfig(configFromC(numSensors, maxSensorValues, chebyPolynomials));
}

CssSensorValues_c CssCommAlgorithm_update(const CssCommAlgorithmHandle* self, const CssSensorValues_c* inputValues) {
    std::array<double, kMaxNumCssSensors> input{};
    std::copy(inputValues->data, inputValues->data + kMaxNumCssSensors, input.begin());

    std::array<double, kMaxNumCssSensors> result = fsw::fromHandle<const ::CssCommAlgorithm>(self)->update(input);

    CssSensorValues_c out{};
    std::copy(result.begin(), result.end(), out.data);
    return out;
}

uint32_t CssCommAlgorithm_getMaxNumCssSensors(void) { return kMaxNumCssSensors; }

uint32_t CssCommAlgorithm_getMaxNumChebyPolys(void) { return kMaxNumChebyPolys; }
