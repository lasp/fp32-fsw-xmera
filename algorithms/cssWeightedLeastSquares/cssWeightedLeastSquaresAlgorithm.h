#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/deviceAvailability.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <math.h>
#include <stdint.h>
#include <Eigen/Core>
#include <array>
#include <optional>

/*! Configuration of a single coarse sun sensor. */
struct CssConfiguration {
    Eigen::Vector3f nHat_B{Eigen::Vector3f::Zero()};  //!< [-] boresight unit vector, body frame components
    //!< [-] state of the sensor; an unavailable sensor takes no part in the fit
    fsw::DeviceAvailability availability{fsw::DeviceAvailability::Available};
};

/*! Estimator products for a single update cycle. */
struct CssWeightedLeastSquaresOutput {
    Eigen::Vector3f sunHeading_B = Eigen::Vector3f::Zero();  //!< [-] estimated unit sun heading, body frame; zero
                                                             //!< when no fit was possible
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();    //!< [r/s] inertial angular velocity, body frame; only
                                                             //!< the component orthogonal to the heading is
                                                             //!< observable, zero without a prior heading or dt
    Eigen::Vector<float, kMaxNumCssSensors> postFitResiduals =
        Eigen::Vector<float, kMaxNumCssSensors>::Zero();  //!< [-] post-fit residuals, one per active sensor, packed
                                                          //!< into the leading numCssViewingSun entries
    uint32_t numCssViewingSun{};                          //!< [-] sensors whose reading exceeded the use threshold
};

/*! @brief Validated configuration for the CSS weighted least squares estimator.

    Boresights are validated as near-unit and stored normalized, so the estimator can rely on exact
    unit vectors. */
class CssWeightedLeastSquaresConfig final {
   public:
    /*! Build a validated configuration.
        @return the validated configuration
        @param cssSensors     [-] boresight and availability of every sensor slot
        @param useMeasurementsAsWeights [-] whether the reading of each sensor becomes its own weight
        @param sensorUseThresh [-] cosine threshold at or below which a reading is discarded
        @param controlPeriod  [s] time between two update() calls, the rate estimate's time step
     */
    static CssWeightedLeastSquaresConfig create(const std::array<CssConfiguration, kMaxNumCssSensors>& cssSensors,
                                                const bool useMeasurementsAsWeights,
                                                const float sensorUseThresh,
                                                const float controlPeriod) {
        if (!isValidCssSensors(cssSensors)) {
            FSW_THROW_INVALID_ARGUMENT(
                "cssWeightedLeastSquares: a minimum of one sensor must be available, and every available "
                "sensor must have a boresight that is a unit vector within 1e-3");
        }
        if (!isValidSensorUseThresh(sensorUseThresh)) {
            FSW_THROW_INVALID_ARGUMENT("cssWeightedLeastSquares: sensorUseThresh must be a cosine in [0, 1]");
        }
        if (!isValidControlPeriod(controlPeriod)) {
            FSW_THROW_INVALID_ARGUMENT("cssWeightedLeastSquares: controlPeriod must be finite and > 0");
        }
        // Pack the configured sensors into the Eigen types the fit works in, normalizing the boresights so
        // downstream code can rely on exact unit vectors. They are validated (near-)unit, so this only
        // removes rounding.
        Eigen::Matrix<float, kMaxNumCssSensors, 3> cssNHat_B = Eigen::Matrix<float, kMaxNumCssSensors, 3>::Zero();
        std::array<fsw::DeviceAvailability, kMaxNumCssSensors> cssAvailability{};
        for (uint32_t i = 0U; i < kMaxNumCssSensors; ++i) {
            cssAvailability.at(i) = cssSensors.at(i).availability;
            if (cssSensors.at(i).availability == fsw::DeviceAvailability::Available) {
                cssNHat_B.row(i) = cssSensors.at(i).nHat_B.stableNormalized().transpose();
            }
        }

        return {cssNHat_B, cssAvailability, useMeasurementsAsWeights, sensorUseThresh, controlPeriod};
    }

    static bool isValidCssSensors(const std::array<CssConfiguration, kMaxNumCssSensors>& cssSensors) {
        bool anyAvailable = false;
        for (uint32_t i = 0; i < kMaxNumCssSensors; ++i) {
            if (cssSensors.at(i).availability != fsw::DeviceAvailability::Available) {
                continue;  // an unavailable sensor never reaches the fit, so its boresight is never used
            }
            anyAvailable = true;
            const Eigen::Vector3f& nHat_B = cssSensors.at(i).nHat_B;
            if (!nHat_B.allFinite() || fabsf(nHat_B.stableNorm() - 1.0F) >= 1e-3F) {
                return false;
            }
        }
        return anyAvailable;
    }

    static bool isValidControlPeriod(const float controlPeriod) {
        return fsw::is_finite(controlPeriod) && controlPeriod > 0.0F;
    }

    /*! A coarse sun sensor cannot report a negative cosine, so a negative threshold cannot exclude any
        reading that zero would not. All it does is admit the sensors that see no sun at all, whose
        readings then constrain the fit as though the heading were square to their boresights. */
    static bool isValidSensorUseThresh(const float sensorUseThresh) {
        return fsw::is_finite(sensorUseThresh) && sensorUseThresh >= 0.0F && sensorUseThresh <= 1.0F;
    }

    // No isValidUseWeights -- a bool with no semantic constraint, the validator would be vacuous.

    const Eigen::Matrix<float, kMaxNumCssSensors, 3>& getCssNHat_B() const { return cssNHat_B; }
    const std::array<fsw::DeviceAvailability, kMaxNumCssSensors>& getCssAvailability() const { return cssAvailability; }
    bool getUseMeasurementsAsWeights() const { return useMeasurementsAsWeights; }
    float getSensorUseThresh() const { return sensorUseThresh; }
    float getControlPeriod() const { return controlPeriod; }

   private:
    CssWeightedLeastSquaresConfig(const Eigen::Matrix<float, kMaxNumCssSensors, 3>& cssNHat_B,
                                  const std::array<fsw::DeviceAvailability, kMaxNumCssSensors>& cssAvailability,
                                  const bool useMeasurementsAsWeights,
                                  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- create() validates by name.
                                  const float sensorUseThresh,
                                  const float controlPeriod)
        : cssNHat_B(cssNHat_B),
          cssAvailability(cssAvailability),
          useMeasurementsAsWeights(useMeasurementsAsWeights),
          sensorUseThresh(sensorUseThresh),
          controlPeriod(controlPeriod) {}

    Eigen::Matrix<float, kMaxNumCssSensors, 3> cssNHat_B = Eigen::Matrix<float, kMaxNumCssSensors, 3>::Zero();
    std::array<fsw::DeviceAvailability, kMaxNumCssSensors> cssAvailability{};
    bool useMeasurementsAsWeights{};
    float sensorUseThresh{};
    float controlPeriod{};
};

/*! @brief Weighted least squares estimator for the body-relative sun heading.

    Fits a sun heading to the readings of a coarse sun sensor constellation. With three or more
    active sensors the fit is a true weighted least squares solution; with two it is the minimum
    norm solution; with one it is the scaled sensor boresight, which is only a guess on the cone of
    possibilities. Two successive heading estimates also yield the inertial angular velocity
    orthogonal to the sun heading. */
class CssWeightedLeastSquaresAlgorithm final {
   public:
    /*! Construct the estimator from a validated configuration.
        @param config the validated configuration to install
     */
    explicit CssWeightedLeastSquaresAlgorithm(const CssWeightedLeastSquaresConfig& config);

    /*! Install a configuration without disturbing runtime state.
        @param config the validated configuration to install
     */
    void setConfig(const CssWeightedLeastSquaresConfig& config);

    /*! Return all runtime state to its post-construction condition. */
    void reInitialize();

    /*! Estimate the sun heading and body rate from one set of CSS readings.
        @return the estimated heading, rate, residuals and the count of sensors viewing the sun
        @param cosValues [-] Per-sensor cosine readings, indexed by sensor
     */
    CssWeightedLeastSquaresOutput update(const Eigen::Vector<float, kMaxNumCssSensors>& cosValues);

   private:
    /*! Solve the least squares fit for the sun heading.
        @return the fit, or nothing when the normal matrix is singular
        @param numCssViewingSun The count on input measurements
        @param weights      The diagonal of the weighting matrix, indexed by sensor. A sensor that takes
                            no part this cycle carries a weight of zero. The values reach the fit only when
                            more than two sensors see the sun, because the one- and two-measurement fits
                            are exactly determined
        @param H            The predicted pointing vector for each measurement, one per row
        @param y            The reading of each sensor, indexed by sensor
     */
    static std::optional<Eigen::Vector3f> computeWlsmn(uint32_t numCssViewingSun,
                                                       const std::array<uint32_t, kMaxNumCssSensors>& activeSensors,
                                                       const Eigen::Vector<float, kMaxNumCssSensors>& weights,
                                                       const Eigen::Matrix<float, kMaxNumCssSensors, 3>& H,
                                                       const Eigen::Vector<float, kMaxNumCssSensors>& y);

    /*! Compute the post-fit residuals for the WLS estimate.
        @return the residuals of the active sensors, packed into the leading numCssViewingSun entries
        @param cssMeas      The measured values for the CSS sensors, indexed by sensor
        @param wlsEst       The WLS estimate computed for the CSS measurements
        @param activeSensors The sensor index behind each observation, in observation order
        @param numCssViewingSun The count on input measurements
     */
    Eigen::Vector<float, kMaxNumCssSensors> computeWlsResiduals(
        const Eigen::Vector<float, kMaxNumCssSensors>& cssMeas,
        const Eigen::Vector3f& wlsEst,
        const std::array<uint32_t, kMaxNumCssSensors>& activeSensors,
        uint32_t numCssViewingSun) const;

    CssWeightedLeastSquaresConfig cfg;                            //!< [-] the validated configuration in force
    Eigen::Vector3f priorSunHeading_B = Eigen::Vector3f::Zero();  //!< [-] prior normalized sun heading, body frame
    bool priorSignalAvailable{};  //!< [-] whether a prior heading is available for the rate
};

#endif
