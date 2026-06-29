#ifndef F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM
#define F32XMERA_BODY_RATE_MISCOMPARE_ALGORITHM

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <Eigen/Core>
#include <cstdint>

/*! @brief Structure containing the body frame angular rate and body rate fault output */
struct BodyRateMiscompareOutput {
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero(); /*!< body frame angular rate */
    bool bodyRateFaultDetected = false;                   /*!< body rate fault */
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
    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    BodyRateMiscompareConfig(float bodyRateThreshold, uint32_t faultPersistenceLimit, bool useImuRates)
        : bodyRateThreshold(bodyRateThreshold),
          faultPersistenceLimit(faultPersistenceLimit),
          useImuRates(useImuRates) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    float bodyRateThreshold;
    uint32_t faultPersistenceLimit;
    bool useImuRates;
};

/*!@brief Module to compare the imu and star tracker rates and fall back to the imu solution if they disagree */
class BodyRateMiscompareAlgorithm final {
   public:
    explicit BodyRateMiscompareAlgorithm(const BodyRateMiscompareConfig& config);
    void setConfig(const BodyRateMiscompareConfig& config);
    void reInitialize();     //!< clears the persistence counter only; a latched fault is preserved
    void reInitializeAll();  //!< clears the persistence counter and re-arms the latched fault from config
    BodyRateMiscompareOutput update(const Eigen::Vector3f& imuOmega_BN_B, const Eigen::Vector3f& stOmega_BN_B);

   private:
    BodyRateMiscompareConfig cfg;
    uint32_t faultPersistenceCount{};
    bool useImuRatesInternal{};  //!< latched fault state; may change without changing the configured useImuRates
};

#endif
