#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
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
    float bias{};                                     //!< [-] calibration scale factor applied to the boresight
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
                                                          //!< into the leading numActiveCss entries
    uint32_t numActiveCss{};                              //!< [-] sensors whose reading exceeded the use threshold
};

/*! @brief Validated configuration for the CSS weighted least squares estimator.

    Boresights are validated as near-unit and stored normalized, so the estimator can rely on exact
    unit vectors. */
class CssWeightedLeastSquaresConfig final {
   public:
    /*! Build a validated configuration.
        @return the validated configuration
        @param cssSensors     [-] boresight and bias of every sensor slot
        @param useWeights     [-] whether to weight the measurements in the least squares fit
        @param sensorUseThresh [-] cosine threshold at or below which a reading is discarded
        @param controlPeriod  [s] time between two update() calls, the rate estimate's time step
     */
    static CssWeightedLeastSquaresConfig create(const std::array<CssConfiguration, kMaxNumCssSensors>& cssSensors,
                                                const bool useWeights,
                                                const float sensorUseThresh,
                                                const float controlPeriod) {
        if (!isValidCssSensors(cssSensors)) {
            FSW_THROW_INVALID_ARGUMENT(
                "cssWeightedLeastSquares: every sensor must have a boresight that is a unit vector within 1e-3 "
                "and a bias that is finite and non-negative");
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
        Eigen::Vector<float, kMaxNumCssSensors> cssBias = Eigen::Vector<float, kMaxNumCssSensors>::Zero();
        for (uint32_t i = 0U; i < kMaxNumCssSensors; ++i) {
            cssNHat_B.row(i) = cssSensors.at(i).nHat_B.stableNormalized().transpose();
            cssBias(i) = cssSensors.at(i).bias;
        }

        return {cssNHat_B, cssBias, useWeights, sensorUseThresh, controlPeriod};
    }

    static bool isValidCssSensors(const std::array<CssConfiguration, kMaxNumCssSensors>& cssSensors) {
        for (uint32_t i = 0; i < kMaxNumCssSensors; ++i) {
            const Eigen::Vector3f& nHat_B = cssSensors.at(i).nHat_B;
            const float bias = cssSensors.at(i).bias;
            if (!nHat_B.allFinite() || fabsf(nHat_B.stableNorm() - 1.0F) >= 1e-3F) {
                return false;
            }
            if (!fsw::is_finite(bias) || bias < 0.0F) {
                return false;
            }
        }
        return true;
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
    const Eigen::Vector<float, kMaxNumCssSensors>& getCssBias() const { return cssBias; }
    bool getUseWeights() const { return useWeights; }
    float getSensorUseThresh() const { return sensorUseThresh; }
    float getControlPeriod() const { return controlPeriod; }

   private:
    CssWeightedLeastSquaresConfig(const Eigen::Matrix<float, kMaxNumCssSensors, 3>& cssNHat_B,
                                  const Eigen::Vector<float, kMaxNumCssSensors>& cssBias,
                                  const bool useWeights,
                                  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) -- create() validates by name.
                                  const float sensorUseThresh,
                                  const float controlPeriod)
        : cssNHat_B(cssNHat_B),
          cssBias(cssBias),
          useWeights(useWeights),
          sensorUseThresh(sensorUseThresh),
          controlPeriod(controlPeriod) {}

    Eigen::Matrix<float, kMaxNumCssSensors, 3> cssNHat_B = Eigen::Matrix<float, kMaxNumCssSensors, 3>::Zero();
    Eigen::Vector<float, kMaxNumCssSensors> cssBias = Eigen::Vector<float, kMaxNumCssSensors>::Zero();
    bool useWeights{};
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
        @return the estimated heading, rate, residuals and active sensor count
        @param cosValues [-] Per-sensor cosine readings, indexed by sensor
     */
    CssWeightedLeastSquaresOutput update(const Eigen::Vector<float, kMaxNumCssSensors>& cosValues);

   private:
    /*! Solve the least squares fit for the sun heading.
        @return the fit, or nothing when the normal matrix is singular
        @param numActiveCss The count on input measurements
        @param weights      The diagonal of the measurement weighting matrix; only applied when more
                            than two measurements are available, as the one- and two-measurement
                            fits are exactly determined
        @param H            The predicted pointing vector for each measurement, one per row
        @param y            The observation vector for the valid sensors
     */
    static std::optional<Eigen::Vector3f> computeWlsmn(uint32_t numActiveCss,
                                                       const Eigen::Vector<float, kMaxNumCssSensors>& weights,
                                                       const Eigen::Matrix<float, kMaxNumCssSensors, 3>& H,
                                                       const Eigen::Vector<float, kMaxNumCssSensors>& y);

    /*! Compute the post-fit residuals for the WLS estimate.
        @return the residuals of the active sensors, packed into the leading numActiveCss entries
        @param cssMeas      The measured values for the CSS sensors
        @param wlsEst       The WLS estimate computed for the CSS measurements
        @param activeSensors The sensor index behind each observation, in observation order
        @param numActiveCss The count on input measurements
     */
    Eigen::Vector<float, kMaxNumCssSensors> computeWlsResiduals(
        const Eigen::Vector<float, kMaxNumCssSensors>& cssMeas,
        const Eigen::Vector3f& wlsEst,
        const std::array<Eigen::Index, kMaxNumCssSensors>& activeSensors,
        uint32_t numActiveCss) const;

    CssWeightedLeastSquaresConfig cfg;                            //!< [-] the validated configuration in force
    Eigen::Vector3f priorSunHeading_B = Eigen::Vector3f::Zero();  //!< [-] prior normalized sun heading, body frame
    bool priorSignalAvailable{};  //!< [-] whether a prior heading is available for the rate
};

#endif
