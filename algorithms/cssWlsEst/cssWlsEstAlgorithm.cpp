#include "cssWlsEstAlgorithm.h"

#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/LU>

/*! Upper limit of the arc-cosine domain. The dot product of two unit vectors can only exceed this
    through round-off, so the principal rotation angle argument is clamped here. */
static constexpr float kMaxPrincipalAngleCosine = 1.0F;

/*! Lower limit of the arc-cosine domain, the negative counterpart of kMaxPrincipalAngleCosine. */
static constexpr float kMinPrincipalAngleCosine = -1.0F;

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
CssWlsEstAlgorithm::CssWlsEstAlgorithm(const CssWlsEstConfig& config) : cfg(config) {
    this->setConfig(config);
    this->reInitialize();
}

/*! Install a configuration. Parameters only; runtime state is left untouched so a reconfiguration
 does not disturb an estimate already in progress.
 @return void
 @param config the validated configuration to install
 */
void CssWlsEstAlgorithm::setConfig(const CssWlsEstConfig& config) { this->cfg = config; }

/*! This method returns all runtime state to its post-construction condition. Local module variables
 that retain time varying states between function calls are reset to their default values.
 @return void
 */
void CssWlsEstAlgorithm::reInitialize() {
    this->priorSignalAvailable = 0;
    this->dOld.setZero();

    /* Reset the prior time flag state.
     If zero, control time step not evaluated on the first function call */
    this->priorTime = 0;
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame, along with the inertial angular velocity
 derived from two successive sun heading estimates
 @return the estimator products for this cycle
 @param callTime The clock time at which the function was called (nanoseconds)
 @param cosValues [-] Per-sensor cosine readings, indexed by sensor
 */
CssWlsEstOutput CssWlsEstAlgorithm::update(const uint64_t callTime, const Eigen::Vector<float, kMaxNumCss>& cosValues) {
    CssWlsEstOutput out;

    /* The predicted pointing vector for each measurement, compacted to the active sensors */
    Eigen::Matrix<float, kMaxNumCss, 3> H = Eigen::Matrix<float, kMaxNumCss, 3>::Zero();
    /* Measurements, compacted to the active sensors */
    Eigen::Vector<float, kMaxNumCss> y = Eigen::Vector<float, kMaxNumCss>::Zero();
    int status = 0; /* Quality of the module estimate */
    float dt;       /* [s] Control update period */

    /*! - Compute control update time */
    if (this->priorTime == 0) {
        dt = 0.0F;
    } else {
        dt = static_cast<float>(static_cast<double>(callTime - this->priorTime) * kNano2Sec);
    }
    this->priorTime = callTime;

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    for (uint32_t i = 0; i < this->cfg.getNumCss(); i = i + 1) {
        const auto sensor = static_cast<Eigen::Index>(i);
        if (cosValues(sensor) > this->cfg.getSensorUseThresh()) {
            const auto active = static_cast<Eigen::Index>(out.numActiveCss);
            H.row(active) = this->cfg.getCssBias()(sensor) * this->cfg.getCssNHat_B().row(sensor);
            y(active) = cosValues(sensor);
            out.numActiveCss = out.numActiveCss + 1;
        }
    }

    /*! Estimation Steps*/
    if (out.numActiveCss == 0) /*! - If there is no sun, just quit*/
    {
        /*! + If no CSS got a strong enough signal.  Sun estimation is not possible.  Return the zero vector instead */
        out.sunHeading_B.setZero();     /* zero the sun heading to indicate no CSS info is available */
        out.omega_BN_B.setZero();       /* zero the rate measure */
        this->priorSignalAvailable = 0; /* reset the prior heading estimate flag */
        out.postFitResiduals = this->computeWlsResiduals(cosValues, out.sunHeading_B);
    } else {
        /*! - If at least one CSS got a strong enough signal.  Proceed with the sun heading estimation */
        /*! -# Configuration option to weight the measurements, otherwise set
         weighting matrix to identity*/
        Eigen::Vector<float, kMaxNumCss> weights = Eigen::Vector<float, kMaxNumCss>::Ones();
        if (this->cfg.getUseWeights()) {
            weights = y;
        }
        /*! -# Get least squares fit for sun pointing vector*/
        status = computeWlsmn(out.numActiveCss, H, weights, y, out.sunHeading_B);
        out.postFitResiduals = this->computeWlsResiduals(cosValues, out.sunHeading_B);

        out.sunHeading_B = out.sunHeading_B.stableNormalized();

        /*! -# Estimate the inertial angular velocity from the rate of the sun heading measurements */
        if (this->priorSignalAvailable && dt > 0.0F) {
            const Eigen::Vector3f dHatNew = out.sunHeading_B.stableNormalized();
            const Eigen::Vector3f dHatOld = this->dOld.stableNormalized();
            out.omega_BN_B = dHatNew.cross(dHatOld).stableNormalized();
            /* compute principal rotation angle between sun heading measurements */
            float dOldDotNew = dHatNew.dot(dHatOld);
            if (dOldDotNew > kMaxPrincipalAngleCosine) dOldDotNew = kMaxPrincipalAngleCosine;
            if (dOldDotNew < kMinPrincipalAngleCosine) dOldDotNew = kMinPrincipalAngleCosine;
            out.omega_BN_B *= safeAcosf(dOldDotNew) / dt;
        } else {
            this->priorSignalAvailable = 1;
        }
        /*! -# Store the sun heading estimate */
        this->dOld = out.sunHeading_B;
    }

    /*! Residual Computation */
    /*! - Capture the heading reported on the filter status output before any anomaly zeroing */
    out.residualStateHeading = out.sunHeading_B;

    /*! Writing Outputs */
    if (status > 0) /*! - If the status from the WLS computation is erroneous, populate the outputs with zeros*/
    {
        /* An error was detected while attempting to compute the sunline direction */
        out.sunHeading_B.setZero();     /* zero the sun heading to indicate anomaly  */
        out.omega_BN_B.setZero();       /* zero the rate measure */
        this->priorSignalAvailable = 0; /* reset the prior heading estimate flag */
    }

    return out;
}

/*! This method computes the post-fit residuals for the WLS estimate.
    @return the per-sensor residuals, zero beyond the configured sensor count
    @param cssMeas The measured values for the CSS sensors
    @param wlsEst The WLS estimate computed for the CSS measurements
*/
Eigen::Vector<float, kMaxNumCss> CssWlsEstAlgorithm::computeWlsResiduals(
    const Eigen::Vector<float, kMaxNumCss>& cssMeas,
    const Eigen::Vector3f& wlsEst) const {
    Eigen::Vector<float, kMaxNumCss> cssResiduals = Eigen::Vector<float, kMaxNumCss>::Zero();

    /*! The method loops through the sensors and performs: */
    for (uint32_t i = 0; i < this->cfg.getNumCss(); i++) {
        const auto sensor = static_cast<Eigen::Index>(i);
        /*! -# A dot product between the computed estimate with each sensor normal */
        const float rawDotProd = wlsEst.dot(this->cfg.getCssNHat_B().row(sensor).transpose());
        /*CSS values can't be negative!*/
        const float cssDotProd = rawDotProd > kMinCssMeasurement ? rawDotProd : kMinCssMeasurement;
        /*! -# A subtraction between that post-fit measurement estimate and the actual measurement*/
        cssResiduals(sensor) = cssMeas(sensor) - cssDotProd;
        /*! -# This populates the post-fit residuals*/
    }

    return cssResiduals;
}

/*! This method computes a least squares fit with the given parameters.
 @return success indicator (0 for good, 1 for fail)
 @param numActiveCss The count on input measurements
 @param H The predicted pointing vector for each measurement, one per row
 @param weights The diagonal of the measurement weighting matrix; only applied when more than two
        measurements are available, as the one- and two-measurement fits are exactly determined
 @param y the observation vector for the valid sensors
 @param x The output least squares fit for the observations
 */
int CssWlsEstAlgorithm::computeWlsmn(const uint32_t numActiveCss,
                                     const Eigen::Matrix<float, kMaxNumCss, 3>& H,
                                     const Eigen::Vector<float, kMaxNumCss>& weights,
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
        const float hhtNorm = hht.norm();
        const float hhtThreshold = kSingularDeterminantRelativeTolerance * hhtNorm * hhtNorm;
        hht.computeInverseAndDetWithCheck(hhtInverse, determinant, invertible, hhtThreshold);
        if (!invertible) {
            hhtInverse.setZero();
            status = 1;
        }
        /*!   -# Multiply the Ht(HHt)^-1 by the observation vector to get fit*/
        x = h.transpose() * hhtInverse * y.head<2>();
    } else if (numActiveCss >= kMinMeasurementsForWeightedFit) { /*! - If we have more than 2, do true LSQ fit*/
        const auto rows = static_cast<Eigen::Index>(numActiveCss);
        /*!    -# Use the weights to compute (HtWH)^-1HtW*/
        const Eigen::Matrix<float, Eigen::Dynamic, 3> h = H.topRows(rows);
        const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w = weights.head(rows).asDiagonal();
        const Eigen::Matrix3f htwh = h.transpose() * w * h;

        Eigen::Matrix3f htwhInverse = Eigen::Matrix3f::Zero();
        float determinant = 0.0F;
        bool invertible = false;
        const float htwhNorm = htwh.norm();
        const float htwhThreshold = kSingularDeterminantRelativeTolerance * htwhNorm * htwhNorm * htwhNorm;
        htwh.computeInverseAndDetWithCheck(htwhInverse, determinant, invertible, htwhThreshold);
        if (!invertible) {
            htwhInverse.setZero();
            status = 1;
        }
        /*!    -# Multiply the LSQ matrix by the obs vector for best fit*/
        x = htwhInverse * h.transpose() * w * y.head(rows);
    }

    return status;
}
