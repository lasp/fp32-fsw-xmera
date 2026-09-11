#include "cssWeightedLeastSquaresAlgorithm.h"

#include "utilities/fsw/safeMath.h"

#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/LU>

/*! Smallest physically meaningful CSS reading. A coarse sun sensor cannot report a negative cosine,
    so the predicted measurement is floored here before differencing against the observation. */
static constexpr float kMinCssMeasurement = 0.0F;

/*! Relative tolerance for treating a normal matrix as singular, sized at a few multiples of the
    working precision's machine epsilon. The determinant of an n-by-n matrix scales as the n-th power
    of the matrix norm, so the absolute threshold handed to Eigen is this factor times the norm
    raised to the matrix dimension, which keeps the test scale invariant. */
static constexpr float kSingularDeterminantRelativeTolerance = 1e-6F;

/*! Number of active measurements below which the fit is exactly determined and the measurement
    weights carry no information. */
static constexpr uint32_t kMinMeasurementsForWeightedFit = 3;

/*! Construct the estimator, installing the configuration and clearing all runtime state.
 @param config the validated configuration to install
 */
CssWeightedLeastSquaresAlgorithm::CssWeightedLeastSquaresAlgorithm(const CssWeightedLeastSquaresConfig& config)
    : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Install a configuration. Parameters only; runtime state is left untouched so a reconfiguration
 does not disturb an estimate already in progress.
 @return void
 @param config the validated configuration to install
 */
void CssWeightedLeastSquaresAlgorithm::setConfig(const CssWeightedLeastSquaresConfig& config) { this->cfg = config; }

/*! This method returns all runtime state to its post-construction condition. Local module variables
 that retain time varying states between function calls are reset to their default values.
 @return void
 */
void CssWeightedLeastSquaresAlgorithm::reInitialize() {
    this->priorSignalAvailable = false;
    this->dOld.setZero();
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame, along with the inertial angular velocity
 derived from two successive sun heading estimates
 @return the estimator products for this cycle
 @param cosValues [-] Per-sensor cosine readings, indexed by sensor
 */
CssWeightedLeastSquaresOutput CssWeightedLeastSquaresAlgorithm::update(
    const Eigen::Vector<float, kMaxNumCss>& cosValues) {
    /* The predicted pointing vector for each measurement, compacted to the active sensors */
    Eigen::Matrix<float, kMaxNumCss, 3> H = Eigen::Matrix<float, kMaxNumCss, 3>::Zero();
    /* Measurements, compacted to the active sensors */
    Eigen::Vector<float, kMaxNumCss> y = Eigen::Vector<float, kMaxNumCss>::Zero();
    /* The sensor index behind each observation, in observation order */
    std::array<Eigen::Index, kMaxNumCss> activeSensors{};
    int status = 0; /* Quality of the module estimate */

    uint32_t numActiveCss = 0;
    Eigen::Vector3f sunHeading_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Vector<float, kMaxNumCss> postFitResiduals = Eigen::Vector<float, kMaxNumCss>::Zero();

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    for (uint32_t i = 0; i < this->cfg.getNumCss(); i = i + 1) {
        if (cosValues(i) > this->cfg.getSensorUseThresh()) {
            H.row(numActiveCss) = this->cfg.getCssBias()(i) * this->cfg.getCssNHat_B().row(i);
            y(numActiveCss) = cosValues(i);
            activeSensors.at(numActiveCss) = i;
            numActiveCss = numActiveCss + 1;
        }
    }

    /*! Estimation Steps*/
    if (numActiveCss == 0) /*! - If there is no sun, just quit*/
    {
        /*! + If no CSS got a strong enough signal.  Sun estimation is not possible.  Return the zero vector instead */
        this->priorSignalAvailable = false; /* reset the prior heading estimate flag */
        postFitResiduals = this->computeWlsResiduals(cosValues, sunHeading_B, activeSensors, numActiveCss);
    } else {
        /*! - If at least one CSS got a strong enough signal.  Proceed with the sun heading estimation */
        /*! -# Configuration option to weight the measurements, otherwise set
         weighting matrix to identity*/
        Eigen::Vector<float, kMaxNumCss> weights = Eigen::Vector<float, kMaxNumCss>::Ones();
        if (this->cfg.getUseWeights()) {
            weights = y;
        }
        /*! -# Get least squares fit for sun pointing vector*/
        status = computeWlsmn(numActiveCss, weights, H, y, sunHeading_B);
        postFitResiduals = this->computeWlsResiduals(cosValues, sunHeading_B, activeSensors, numActiveCss);

        sunHeading_B = sunHeading_B.stableNormalized();

        /*! -# Estimate the inertial angular velocity from the rate of the sun heading measurements */
        if (this->priorSignalAvailable) {
            const Eigen::Vector3f dHatNew = sunHeading_B.stableNormalized();
            const Eigen::Vector3f dHatOld = this->dOld.stableNormalized();
            omega_BN_B = dHatNew.cross(dHatOld).stableNormalized();
            /* compute principal rotation angle between sun heading measurements */
            const float principalAngle = safeAcosf(dHatNew.dot(dHatOld));
            omega_BN_B *= principalAngle / this->cfg.getControlPeriod();
        } else {
            this->priorSignalAvailable = true;
        }
        /*! -# Store the sun heading estimate */
        this->dOld = sunHeading_B;
    }

    /*! - Capture the heading reported on the filter status output before any anomaly zeroing */
    const Eigen::Vector3f residualStateHeading = sunHeading_B;

    if (status > 0) /*! - If the status from the WLS computation is erroneous, populate the outputs with zeros*/
    {
        /* An error was detected while attempting to compute the sunline direction */
        sunHeading_B.setZero();             /* zero the sun heading to indicate anomaly  */
        omega_BN_B.setZero();               /* zero the rate measure */
        this->priorSignalAvailable = false; /* reset the prior heading estimate flag */
    }

    return {.sunHeading_B = sunHeading_B,
            .omega_BN_B = omega_BN_B,
            .residualStateHeading = residualStateHeading,
            .postFitResiduals = postFitResiduals,
            .numActiveCss = numActiveCss};
}

/*! This method computes the post-fit residuals for the WLS estimate. The residuals are indexed by
    observation rather than by sensor slot, so the leading numActiveCss entries carry the sensors that
    contributed to the fit and the remainder stay zero.
    @return the residuals of the active sensors, packed into the leading numActiveCss entries
    @param cssMeas The measured values for the CSS sensors
    @param wlsEst The WLS estimate computed for the CSS measurements
    @param activeSensors The sensor index behind each observation, in observation order
    @param numActiveCss The count on input measurements
*/
Eigen::Vector<float, kMaxNumCss> CssWeightedLeastSquaresAlgorithm::computeWlsResiduals(
    const Eigen::Vector<float, kMaxNumCss>& cssMeas,
    const Eigen::Vector3f& wlsEst,
    const std::array<Eigen::Index, kMaxNumCss>& activeSensors,
    const uint32_t numActiveCss) const {
    Eigen::Vector<float, kMaxNumCss> cssResiduals = Eigen::Vector<float, kMaxNumCss>::Zero();

    /*! The method loops through the observations and performs: */
    for (uint32_t observation = 0; observation < numActiveCss; observation++) {
        const Eigen::Index sensor = activeSensors.at(observation);
        /*! -# A dot product between the computed estimate with each sensor normal */
        const float rawDotProd = wlsEst.dot(this->cfg.getCssNHat_B().row(sensor).transpose());
        /*CSS values can't be negative!*/
        const float cssDotProd = rawDotProd > kMinCssMeasurement ? rawDotProd : kMinCssMeasurement;
        /*! -# A subtraction between that post-fit measurement estimate and the actual measurement*/
        cssResiduals(observation) = cssMeas(sensor) - cssDotProd;
        /*! -# This populates the post-fit residuals*/
    }

    return cssResiduals;
}

/*! This method computes a least squares fit with the given parameters.
 @return success indicator (0 for good, 1 for fail)
 @param numActiveCss The count on input measurements
 @param weights The diagonal of the measurement weighting matrix; only applied when more than two
        measurements are available, as the one- and two-measurement fits are exactly determined
 @param H The predicted pointing vector for each measurement, one per row
 @param y the observation vector for the valid sensors
 @param x The output least squares fit for the observations
 */
int CssWeightedLeastSquaresAlgorithm::computeWlsmn(const uint32_t numActiveCss,
                                                   const Eigen::Vector<float, kMaxNumCss>& weights,
                                                   const Eigen::Matrix<float, kMaxNumCss, 3>& H,
                                                   const Eigen::Vector<float, kMaxNumCss>& y,
                                                   Eigen::Vector3f& x) {
    int status = 0;

    /*! - If we only have one sensor, output best guess (cone of possiblities)*/
    if (numActiveCss == 1) {
        /* Here's a guess.  Do with it what you will. */
        x = H.row(0).transpose() * y(0);
    } else if (numActiveCss == 2) { /*! - If we have two, then do a 2x2 fit */
        /*!   -# Find minimum norm solution */
        const Eigen::Matrix<float, 2, 3> h = H.topRows<2>();
        const Eigen::Matrix2f hht = h * h.transpose();

        Eigen::Matrix2f hhtInverse = Eigen::Matrix2f::Zero();
        float determinant = 0.0F;
        bool invertible = false;
        const float hhtNorm = hht.stableNorm();
        const float hhtThreshold = kSingularDeterminantRelativeTolerance * hhtNorm * hhtNorm;
        hht.computeInverseAndDetWithCheck(hhtInverse, determinant, invertible, hhtThreshold);
        if (!invertible) {
            hhtInverse.setZero();
            status = 1;
        }
        /*!   -# Multiply the Ht(HHt)^-1 by the observation vector to get fit*/
        x = h.transpose() * hhtInverse * y.head<2>();
    } else if (numActiveCss >= kMinMeasurementsForWeightedFit) { /*! - If we have more than 2, do true LSQ fit*/
        /*!    -# Use the weights to compute (HtWH)^-1HtW. The rows of H and the entries of y past
           numActiveCss are zero, so the products over the full operands equal the products over the
           active measurements alone. Forming them at full size keeps every intermediate a
           fixed-size Eigen type; a dynamically sized one would allocate, and this build forbids
           heap allocation. */
        const Eigen::Matrix<float, kMaxNumCss, 3> wh = weights.asDiagonal() * H;
        const Eigen::Matrix3f htwh = H.transpose() * wh;

        Eigen::Matrix3f htwhInverse = Eigen::Matrix3f::Zero();
        float determinant = 0.0F;
        bool invertible = false;
        const float htwhNorm = htwh.stableNorm();
        const float htwhThreshold = kSingularDeterminantRelativeTolerance * htwhNorm * htwhNorm * htwhNorm;
        htwh.computeInverseAndDetWithCheck(htwhInverse, determinant, invertible, htwhThreshold);
        if (!invertible) {
            htwhInverse.setZero();
            status = 1;
        }
        /*!    -# Multiply the LSQ matrix by the obs vector for best fit*/
        x = htwhInverse * (wh.transpose() * y);
    }

    return status;
}
