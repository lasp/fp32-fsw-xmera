#include "cssCommAlgorithm_c.h"
#include "cssCommAlgorithm.h"

#include <algorithm>
#include <array>

namespace {
CssCommConfig configFromC(const CssCommConfig_c& c) {
    std::array<double, kMaxNumChebyPolys> chebyPolynomials{};
    std::copy(c.chebyPolynomials, c.chebyPolynomials + kMaxNumChebyPolys, chebyPolynomials.begin());
    std::array<double, kMaxNumCssSensors> maxSensorValues{};
    std::copy(c.maxSensorValues, c.maxSensorValues + kMaxNumCssSensors, maxSensorValues.begin());
    return CssCommConfig::create(c.numSensors, maxSensorValues, c.chebyCount, chebyPolynomials);
}
}  // namespace

CssCommAlgorithmHandle* CssCommAlgorithm_create(const CssCommConfig_c* config) {
    return reinterpret_cast<CssCommAlgorithmHandle*>(new ::CssCommAlgorithm(configFromC(*config)));
}

void CssCommAlgorithm_destroy(CssCommAlgorithmHandle* self) { delete reinterpret_cast<::CssCommAlgorithm*>(self); }

void CssCommAlgorithm_setConfig(CssCommAlgorithmHandle* self, const CssCommConfig_c* config) {
    reinterpret_cast<::CssCommAlgorithm*>(self)->setConfig(configFromC(*config));
}

CssSensorValues_c CssCommAlgorithm_update(const CssCommAlgorithmHandle* self, const CssSensorValues_c* inputValues) {
    std::array<double, kMaxNumCssSensors> input{};
    std::copy(inputValues->data, inputValues->data + kMaxNumCssSensors, input.begin());

    std::array<double, kMaxNumCssSensors> result = reinterpret_cast<const ::CssCommAlgorithm*>(self)->update(input);

    CssSensorValues_c out{};
    std::copy(result.begin(), result.end(), out.data);
    return out;
}

uint32_t CssCommAlgorithm_getMaxNumCssSensors(void) { return kMaxNumCssSensors; }

uint32_t CssCommAlgorithm_getMaxNumChebyPolys(void) { return kMaxNumChebyPolys; }
