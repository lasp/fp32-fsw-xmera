#include "cssCommAlgorithm_c.h"
#include "cssCommAlgorithm.h"

#include <algorithm>

CssCommAlgorithmHandle* CssCommAlgorithm_create(void) {
    return reinterpret_cast<CssCommAlgorithmHandle*>(new ::CssCommAlgorithm());
}

void CssCommAlgorithm_destroy(CssCommAlgorithmHandle* self) { delete reinterpret_cast<::CssCommAlgorithm*>(self); }

CssSensorValues_c CssCommAlgorithm_update(const CssCommAlgorithmHandle* self, const CssSensorValues_c* inputValues) {
    std::array<double, kMaxNumCssSensors> input{};
    std::copy(inputValues->data, inputValues->data + kMaxNumCssSensors, input.begin());

    std::array<double, kMaxNumCssSensors> result = reinterpret_cast<const ::CssCommAlgorithm*>(self)->update(input);

    CssSensorValues_c out{};
    std::copy(result.begin(), result.end(), out.data);
    return out;
}

void CssCommAlgorithm_setNumSensors(CssCommAlgorithmHandle* self, const uint32_t numberOfSensors) {
    reinterpret_cast<::CssCommAlgorithm*>(self)->setNumSensors(numberOfSensors);
}

uint32_t CssCommAlgorithm_getNumSensors(const CssCommAlgorithmHandle* self) {
    return reinterpret_cast<const ::CssCommAlgorithm*>(self)->getNumSensors();
}

void CssCommAlgorithm_setMaxSensorValue(CssCommAlgorithmHandle* self, const double maxValue) {
    reinterpret_cast<::CssCommAlgorithm*>(self)->setMaxSensorValue(maxValue);
}

double CssCommAlgorithm_getMaxSensorValue(const CssCommAlgorithmHandle* self) {
    return reinterpret_cast<const ::CssCommAlgorithm*>(self)->getMaxSensorValue();
}

void CssCommAlgorithm_setChebyCount(CssCommAlgorithmHandle* self, const uint32_t count) {
    reinterpret_cast<::CssCommAlgorithm*>(self)->setChebyCount(count);
}

uint32_t CssCommAlgorithm_getChebyCount(const CssCommAlgorithmHandle* self) {
    return reinterpret_cast<const ::CssCommAlgorithm*>(self)->getChebyCount();
}

void CssCommAlgorithm_setChebyPolynomials(CssCommAlgorithmHandle* self, const ChebyPolynomials_c* polynomials) {
    std::array<double, kMaxNumChebyPolys> arr{};
    std::copy(polynomials->data, polynomials->data + kMaxNumChebyPolys, arr.begin());
    reinterpret_cast<::CssCommAlgorithm*>(self)->setChebyPolynomials(arr);
}

ChebyPolynomials_c CssCommAlgorithm_getChebyPolynomials(const CssCommAlgorithmHandle* self) {
    std::array<double, kMaxNumChebyPolys> arr =
        reinterpret_cast<const ::CssCommAlgorithm*>(self)->getChebyPolynomials();
    ChebyPolynomials_c out{};
    std::copy(arr.begin(), arr.end(), out.data);
    return out;
}

uint32_t CssCommAlgorithm_getMaxNumCssSensors(void) { return kMaxNumCssSensors; }

uint32_t CssCommAlgorithm_getMaxNumChebyPolys(void) { return kMaxNumChebyPolys; }
