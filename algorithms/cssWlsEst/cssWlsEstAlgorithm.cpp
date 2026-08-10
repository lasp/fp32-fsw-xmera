#include "cssWlsEstAlgorithm.h"

#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/safeMath.h>

#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/LU>

/*! Upper limit of the arc-cosine domain. The dot product of two unit vectors can only exceed this
    through round-off, so the principal rotation angle argument is clamped here. */
static constexpr double kMaxPrincipalAngleCosine = 1.0;

/*! Lower limit of the arc-cosine domain, the negative counterpart of kMaxPrincipalAngleCosine. */
static constexpr double kMinPrincipalAngleCosine = -1.0;

/*! Smallest physically meaningful CSS reading. A coarse sun sensor cannot report a negative cosine,
    so the predicted measurement is floored here before differencing against the observation. */
static constexpr double kMinCssMeasurement = 0.0;

/*! Number of active measurements below which the fit is exactly determined and the measurement
    weights carry no information. */
static constexpr uint32_t kMinMeasurementsForWeightedFit = 3;

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
CssWlsEstOutput CssWlsEstAlgorithm::update(const uint64_t callTime,
                                           const Eigen::Vector<double, kMaxNumCss>& cosValues) {
    CssWlsEstOutput out;

    /* The predicted pointing vector for each measurement, compacted to the active sensors */
    Eigen::Matrix<double, kMaxNumCss, 3> H = Eigen::Matrix<double, kMaxNumCss, 3>::Zero();
    /* Measurements, compacted to the active sensors */
    Eigen::Vector<double, kMaxNumCss> y = Eigen::Vector<double, kMaxNumCss>::Zero();
    int status = 0; /* Quality of the module estimate */
    double dt;      /* [s] Control update period */

    /*! - Compute control update time */
    if (this->priorTime == 0) {
        dt = 0.0;
    } else {
        dt = static_cast<double>(callTime - this->priorTime) * NANO2SEC;
    }
    this->priorTime = callTime;

    /*! - Loop over the maximum number of sensors to check for good measurements */
    /*! -# Isolate if measurement is good */
    /*! -# Set body vector for this measurement */
    /*! -# Get measurement value into observation vector */
    /*! -# increase the number of valid observations */
    /*! -# Otherwise just continue */
    for (uint32_t i = 0; i < this->numCss; i = i + 1) {
        const auto sensor = static_cast<Eigen::Index>(i);
        if (cosValues(sensor) > this->sensorUseThresh) {
            const auto active = static_cast<Eigen::Index>(out.numActiveCss);
            H.row(active) = this->cssBias(sensor) * this->cssNHat_B.row(sensor);
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
        Eigen::Vector<double, kMaxNumCss> weights = Eigen::Vector<double, kMaxNumCss>::Ones();
        if (this->useWeights > 0) {
            weights = y;
        }
        /*! -# Get least squares fit for sun pointing vector*/
        status = computeWlsmn(out.numActiveCss, H, weights, y, out.sunHeading_B);
        out.postFitResiduals = this->computeWlsResiduals(cosValues, out.sunHeading_B);

        out.sunHeading_B = out.sunHeading_B.stableNormalized();

        /*! -# Estimate the inertial angular velocity from the rate of the sun heading measurements */
        if (this->priorSignalAvailable && dt > 0.0) {
            const Eigen::Vector3d dHatNew = out.sunHeading_B.stableNormalized();
            const Eigen::Vector3d dHatOld = this->dOld.stableNormalized();
            out.omega_BN_B = dHatNew.cross(dHatOld).stableNormalized();
            /* compute principal rotation angle between sun heading measurements */
            double dOldDotNew = dHatNew.dot(dHatOld);
            if (dOldDotNew > kMaxPrincipalAngleCosine) dOldDotNew = kMaxPrincipalAngleCosine;
            if (dOldDotNew < kMinPrincipalAngleCosine) dOldDotNew = kMinPrincipalAngleCosine;
            out.omega_BN_B *= safeAcos(dOldDotNew) / dt;
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
Eigen::Vector<double, kMaxNumCss> CssWlsEstAlgorithm::computeWlsResiduals(
    const Eigen::Vector<double, kMaxNumCss>& cssMeas,
    const Eigen::Vector3d& wlsEst) const {
    Eigen::Vector<double, kMaxNumCss> cssResiduals = Eigen::Vector<double, kMaxNumCss>::Zero();

    /*! The method loops through the sensors and performs: */
    for (uint32_t i = 0; i < this->numCss; i++) {
        const auto sensor = static_cast<Eigen::Index>(i);
        /*! -# A dot product between the computed estimate with each sensor normal */
        const double rawDotProd = wlsEst.dot(this->cssNHat_B.row(sensor).transpose());
        /*CSS values can't be negative!*/
        const double cssDotProd = rawDotProd > kMinCssMeasurement ? rawDotProd : kMinCssMeasurement;
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
                                     const Eigen::Matrix<double, kMaxNumCss, 3>& H,
                                     const Eigen::Vector<double, kMaxNumCss>& weights,
                                     const Eigen::Vector<double, kMaxNumCss>& y,
                                     Eigen::Vector3d& x) {
    int status = 0;

    /*! - If we only have one sensor, output best guess (cone of possiblities)*/
    if (numActiveCss == 1) {
        /* Here's a guess.  Do with it what you will. */
        x = H.row(0).transpose() * y(0);
    } else if (numActiveCss == 2) { /*! - If we have two, then do a 2x2 fit */
        /*!   -# Find minimum norm solution */
        const Eigen::Matrix<double, 2, 3> h = H.topRows<2>();
        const Eigen::Matrix2d hht = h * h.transpose();

        Eigen::Matrix2d hhtInverse = Eigen::Matrix2d::Zero();
        double determinant = 0.0;
        bool invertible = false;
        hht.computeInverseAndDetWithCheck(hhtInverse, determinant, invertible);
        if (!invertible) {
            hhtInverse.setZero();
            status = 1;
        }
        /*!   -# Multiply the Ht(HHt)^-1 by the observation vector to get fit*/
        x = h.transpose() * hhtInverse * y.head<2>();
    } else if (numActiveCss >= kMinMeasurementsForWeightedFit) { /*! - If we have more than 2, do true LSQ fit*/
        const auto rows = static_cast<Eigen::Index>(numActiveCss);
        /*!    -# Use the weights to compute (HtWH)^-1HtW*/
        const Eigen::Matrix<double, Eigen::Dynamic, 3> h = H.topRows(rows);
        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> w = weights.head(rows).asDiagonal();
        const Eigen::Matrix3d htwh = h.transpose() * w * h;

        Eigen::Matrix3d htwhInverse = Eigen::Matrix3d::Zero();
        double determinant = 0.0;
        bool invertible = false;
        htwh.computeInverseAndDetWithCheck(htwhInverse, determinant, invertible);
        if (!invertible) {
            htwhInverse.setZero();
            status = 1;
        }
        /*!    -# Multiply the LSQ matrix by the obs vector for best fit*/
        x = htwhInverse * h.transpose() * w * y.head(rows);
    }

    return status;
}
