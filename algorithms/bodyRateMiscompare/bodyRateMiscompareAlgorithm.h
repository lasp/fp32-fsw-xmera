#ifndef F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM
#define F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <Eigen/Core>
#include <cstdint>

/*! @brief Structure containing the body frame angular rate and body rate fault output */
struct BodyRateMiscompareOutput {
    Eigen::Vector3f omega_BN_B; /*!< body frame angular rate */
    bool bodyRateFaultDetected; /*!< body rate fault */
};

/*!
 * @brief Validated configuration for the body rate miscompare algorithm.
 *
 * An instance can only exist with a finite, strictly positive body rate threshold and a strictly
 * positive fault persistence limit. Construct via BodyRateMiscompareConfig::create(...).
 */
class BodyRateMiscompareConfig final {
   public:
    static BodyRateMiscompareConfig create(float bodyRateThreshold, uint32_t faultPersistenceLimit, bool useImuRates) {
        if (!isValidBodyRateThreshold(bodyRateThreshold)) {
            FSW_THROW_INVALID_ARGUMENT("bodyRateMiscompare: bodyRateThreshold must be finite and > 0");
        }
        if (!isValidFaultPersistenceLimit(faultPersistenceLimit)) {
            FSW_THROW_INVALID_ARGUMENT("bodyRateMiscompare: faultPersistenceLimit must be > 0");
        }
        return {bodyRateThreshold, faultPersistenceLimit, useImuRates};
    }

    static bool isValidBodyRateThreshold(float bodyRateThreshold) {
        return fsw::is_finite(bodyRateThreshold) && bodyRateThreshold > 0.0F;
    }
    static bool isValidFaultPersistenceLimit(uint32_t faultPersistenceLimit) { return faultPersistenceLimit > 0U; }
    // No isValidUseImuRates — any bool value is valid.

    float getBodyRateThreshold() const { return bodyRateThreshold; }
    uint32_t getFaultPersistenceLimit() const { return faultPersistenceLimit; }
    bool getUseImuRates() const { return useImuRates; }

   private:
    BodyRateMiscompareConfig(float bodyRateThreshold, uint32_t faultPersistenceLimit, bool useImuRates)
        : bodyRateThreshold(bodyRateThreshold),
          faultPersistenceLimit(faultPersistenceLimit),
          useImuRates(useImuRates) {}

    float bodyRateThreshold;
    uint32_t faultPersistenceLimit;
    bool useImuRates;
};

/*!@brief Module to compare the imu and star tracker rates and fall back to the imu solution if they disagree */
class BodyRateMiscompareAlgorithm {
   public:
    void reset();
    BodyRateMiscompareOutput update(const Eigen::Vector3f& imuOmega_BN_B, const Eigen::Vector3f& stOmega_BN_B);
    void setBodyRateThreshold(float bodyRateThresholdIn);
    float getBodyRateThreshold() const;
    void setFaultPersistenceLimit(uint32_t faultPersistenceLimitIn);
    uint32_t getFaultPersistenceLimit() const;
    void setUseImuRates(bool useImuRatesIn);
    bool getUseImuRates() const;

   private:
    float bodyRateThreshold{};            // rate threshold to trigger body rate miscompare fault
    uint32_t faultPersistenceLimit = 1U;  // number of consecutive update calls needed to trigger the fault
    bool useImuRates{};                   // force to use IMU rates, even if rates agree and no fault is triggered

    uint32_t faultPersistenceCount{};
    bool useImuRatesInternal{};  // this separate variable can change without changing the settable parameter
};

#endif
