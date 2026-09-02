#ifndef F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_H
#define F32XMERA_CSS_WEIGHTED_LEAST_SQUARES_ALGORITHM_H

#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <math.h>
#include <stdint.h>
#include <Eigen/Core>

/*! Maximum number of coarse sun sensors the estimator can process in one cycle, fixed by the bound on
    the CSS array measurement message. */
inline constexpr int kMaxNumCss = MAX_NUM_CSS_SENSORS;

/*! Estimator products for a single update cycle. */
struct CssWeightedLeastSquaresOutput {
    Eigen::Vector3f sunHeading_B = Eigen::Vector3f::Zero();  //!< [-] estimated unit sun heading, body frame; zero
                                                             //!< when no fit was possible
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();    //!< [r/s] inertial angular velocity, body frame; only
                                                             //!< the component orthogonal to the heading is
                                                             //!< observable, zero without a prior heading or dt
    Eigen::Vector3f residualStateHeading = Eigen::Vector3f::Zero();  //!< [-] heading on the filter status output,
                                                                     //!< captured before the singular-fit zeroing
    Eigen::Vector<float, kMaxNumCss> postFitResiduals =
        Eigen::Vector<float, kMaxNumCss>::Zero();  //!< [-] post-fit residuals, one per configured sensor
    uint32_t numActiveCss{};                       //!< [-] sensors whose reading exceeded the use threshold
};

/*! @brief Validated configuration for the CSS weighted least squares estimator.

    Boresights are validated as near-unit and stored normalized, so the estimator can rely on exact
    unit vectors. */
class CssWeightedLeastSquaresConfig final {
   public:
    /*! Build a validated configuration.
        @return the validated configuration
        @param cssNHat_B      [-] per-sensor boresight unit vectors in body frame, one sensor per row
        @param cssBias        [-] per-sensor calibration scale factors
        @param numCss         [-] number of configured sensors
        @param useWeights     [-] whether to weight the measurements in the least squares fit
        @param sensorUseThresh [-] cosine threshold at or below which a reading is discarded
     */
    static CssWeightedLeastSquaresConfig create(const Eigen::Matrix<float, kMaxNumCss, 3>& cssNHat_B,
                                                const Eigen::Vector<float, kMaxNumCss>& cssBias,
                                                const uint32_t numCss,
                                                const bool useWeights,
                                                const float sensorUseThresh) {
        if (!isValidNumCss(numCss)) {
            FSW_THROW_INVALID_ARGUMENT("cssWeightedLeastSquares: numCss must be in [1, kMaxNumCss]");
        }
        if (!isValidCssNHat_B(cssNHat_B, numCss)) {
            FSW_THROW_INVALID_ARGUMENT(
                "cssWeightedLeastSquares: the first numCss cssNHat_B rows must be unit vectors within 1e-3");
        }
        if (!isValidCssBias(cssBias, numCss)) {
            FSW_THROW_INVALID_ARGUMENT(
                "cssWeightedLeastSquares: the first numCss cssBias entries must be finite and non-negative");
        }
        if (!isValidSensorUseThresh(sensorUseThresh)) {
            FSW_THROW_INVALID_ARGUMENT("cssWeightedLeastSquares: sensorUseThresh must be a cosine in [-1, 1]");
        }
        // Normalize the boresights so downstream code can rely on exact unit vectors. The rows are validated
        // (near-)unit, so this only removes rounding; rows beyond numCss are unused and stay zero.
        Eigen::Matrix<float, kMaxNumCss, 3> normalizedCssNHat_B = cssNHat_B;
        for (uint32_t i = 0U; i < numCss; ++i) {
            normalizedCssNHat_B.row(static_cast<Eigen::Index>(i)).normalize();
        }

        return {normalizedCssNHat_B, cssBias, numCss, useWeights, sensorUseThresh};
    }

    static bool isValidNumCss(const uint32_t numCss) {
        return numCss >= 1U && numCss <= static_cast<uint32_t>(kMaxNumCss);
    }

    static bool isValidCssNHat_B(const Eigen::Matrix<float, kMaxNumCss, 3>& cssNHat_B, const uint32_t numCss) {
        if (!isValidNumCss(numCss)) {
            return false;
        }
        for (uint32_t i = 0; i < numCss; ++i) {
            const Eigen::Vector3f row = cssNHat_B.row(static_cast<Eigen::Index>(i)).transpose();
            if (!row.allFinite() || fabsf(row.stableNorm() - 1.0F) >= 1e-3F) {
                return false;
            }
        }
        return true;
    }

    static bool isValidCssBias(const Eigen::Vector<float, kMaxNumCss>& cssBias, const uint32_t numCss) {
        if (!isValidNumCss(numCss)) {
            return false;
        }
        for (uint32_t i = 0; i < numCss; ++i) {
            const float bias = cssBias(static_cast<Eigen::Index>(i));
            if (!fsw::is_finite(bias) || bias < 0.0F) {
                return false;
            }
        }
        return true;
    }

    static bool isValidSensorUseThresh(const float sensorUseThresh) {
        return fsw::is_finite(sensorUseThresh) && sensorUseThresh >= -1.0F && sensorUseThresh <= 1.0F;
    }

    // No isValidUseWeights -- a bool with no semantic constraint, the validator would be vacuous.

    const Eigen::Matrix<float, kMaxNumCss, 3>& getCssNHat_B() const { return cssNHat_B; }
    const Eigen::Vector<float, kMaxNumCss>& getCssBias() const { return cssBias; }
    uint32_t getNumCss() const { return numCss; }
    bool getUseWeights() const { return useWeights; }
    float getSensorUseThresh() const { return sensorUseThresh; }

   private:
    CssWeightedLeastSquaresConfig(const Eigen::Matrix<float, kMaxNumCss, 3>& cssNHat_B,
                                  const Eigen::Vector<float, kMaxNumCss>& cssBias,
                                  const uint32_t numCss,
                                  const bool useWeights,
                                  const float sensorUseThresh)
        : cssNHat_B(cssNHat_B),
          cssBias(cssBias),
          numCss(numCss),
          useWeights(useWeights),
          sensorUseThresh(sensorUseThresh) {}

    Eigen::Matrix<float, kMaxNumCss, 3> cssNHat_B = Eigen::Matrix<float, kMaxNumCss, 3>::Zero();
    Eigen::Vector<float, kMaxNumCss> cssBias = Eigen::Vector<float, kMaxNumCss>::Zero();
    uint32_t numCss{};
    bool useWeights{};
    float sensorUseThresh{};
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
        @param callTime  The clock time at which the function was called (nanoseconds)
        @param cosValues [-] Per-sensor cosine readings, indexed by sensor
     */
    CssWeightedLeastSquaresOutput update(uint64_t callTime, const Eigen::Vector<float, kMaxNumCss>& cosValues);

   private:
    /*! Solve the least squares fit for the sun heading.
        @return success indicator (0 for good, 1 for a singular normal matrix)
        @param numActiveCss The count on input measurements
        @param weights      The diagonal of the measurement weighting matrix; only applied when more
                            than two measurements are available, as the one- and two-measurement
                            fits are exactly determined
        @param H            The predicted pointing vector for each measurement, one per row
        @param y            The observation vector for the valid sensors
        @param x            The output least squares fit for the observations
     */
    static int computeWlsmn(uint32_t numActiveCss,
                            const Eigen::Vector<float, kMaxNumCss>& weights,
                            const Eigen::Matrix<float, kMaxNumCss, 3>& H,
                            const Eigen::Vector<float, kMaxNumCss>& y,
                            Eigen::Vector3f& x);

    /*! Compute the post-fit residuals for the WLS estimate.
        @return the per-sensor residuals, zero beyond the configured sensor count
        @param cssMeas The measured values for the CSS sensors
        @param wlsEst  The WLS estimate computed for the CSS measurements
     */
    Eigen::Vector<float, kMaxNumCss> computeWlsResiduals(const Eigen::Vector<float, kMaxNumCss>& cssMeas,
                                                         const Eigen::Vector3f& wlsEst) const;

    CssWeightedLeastSquaresConfig cfg;               //!< [-] the validated configuration in force
    Eigen::Vector3f dOld = Eigen::Vector3f::Zero();  //!< [-] prior normalized sun heading, body frame
    uint32_t priorSignalAvailable{};                 //!< [-] whether a prior heading is available for the rate
    uint64_t priorTime{};                            //!< [ns] previous update time; zero until the first call
};

#endif
