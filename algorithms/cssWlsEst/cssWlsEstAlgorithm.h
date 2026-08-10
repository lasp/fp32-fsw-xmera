#ifndef F32XMERA_CSS_WLS_EST_ALGORITHM_H
#define F32XMERA_CSS_WLS_EST_ALGORITHM_H

#include <stdint.h>
#include <Eigen/Core>

/*! Maximum number of coarse sun sensors the estimator can process in one cycle. The adapter asserts
    that this matches the bound on the CSS array measurement message. */
inline constexpr int kMaxNumCss = 32;

/*! Estimator products for a single update cycle. */
struct CssWlsEstOutput {
    /*! [-] Estimated unit sun heading, body frame components. Zero when no fit was possible. */
    Eigen::Vector3d sunHeading_B = Eigen::Vector3d::Zero();

    /*! [r/s] Inertial angular velocity, body frame components. Only the component orthogonal to the
        sun heading is observable; zero when no prior heading or no elapsed time is available. */
    Eigen::Vector3d omega_BN_B = Eigen::Vector3d::Zero();

    /*! [-] Sun heading as reported on the filter status output. Captured before the singular-fit
        zeroing, so on a degenerate fit it retains the raw estimate for diagnostics while
        sunHeading_B is zero. */
    Eigen::Vector3d residualStateHeading = Eigen::Vector3d::Zero();

    /*! [-] Post-fit measurement residuals, one entry per configured sensor. Entries beyond the
        configured sensor count stay zero. */
    Eigen::Vector<double, kMaxNumCss> postFitResiduals = Eigen::Vector<double, kMaxNumCss>::Zero();

    /*! [-] Number of sensors whose reading exceeded the use threshold this cycle. */
    uint32_t numActiveCss{};
};

/*! @brief Weighted least squares estimator for the body-relative sun heading.

    Fits a sun heading to the readings of a coarse sun sensor constellation. With three or more
    active sensors the fit is a true weighted least squares solution; with two it is the minimum
    norm solution; with one it is the scaled sensor boresight, which is only a guess on the cone of
    possibilities. Two successive heading estimates also yield the inertial angular velocity
    orthogonal to the sun heading. */
class CssWlsEstAlgorithm final {
   public:
    /*! Return all runtime state to its post-construction condition. */
    void reInitialize();

    /*! Estimate the sun heading and body rate from one set of CSS readings.
        @return the estimated heading, rate, residuals and active sensor count
        @param callTime  The clock time at which the function was called (nanoseconds)
        @param cosValues [-] Per-sensor cosine readings, indexed by sensor
     */
    CssWlsEstOutput update(uint64_t callTime, const Eigen::Vector<double, kMaxNumCss>& cosValues);

    /*! [-] Per-sensor boresight unit vectors in body frame components, one sensor per row. */
    Eigen::Matrix<double, kMaxNumCss, 3> cssNHat_B = Eigen::Matrix<double, kMaxNumCss, 3>::Zero();

    /*! [-] Per-sensor calibration scale factor applied to the sensor boresight. */
    Eigen::Vector<double, kMaxNumCss> cssBias = Eigen::Vector<double, kMaxNumCss>::Zero();

    /*! [-] Number of configured sensors; only the leading entries of the arrays above are read. */
    uint32_t numCss{};

    /*! [-] Flag selecting measurement weighting for the least squares fit. */
    uint32_t useWeights{};

    /*! [-] Cosine threshold at or below which a sensor reading is discarded. */
    double sensorUseThresh{};

   private:
    /*! Solve the least squares fit for the sun heading.
        @return success indicator (0 for good, 1 for a singular normal matrix)
        @param numActiveCss The count on input measurements
        @param H            The predicted pointing vector for each measurement, one per row
        @param weights      The diagonal of the measurement weighting matrix; only applied when more
                            than two measurements are available, as the one- and two-measurement
                            fits are exactly determined
        @param y            The observation vector for the valid sensors
        @param x            The output least squares fit for the observations
     */
    static int computeWlsmn(uint32_t numActiveCss,
                            const Eigen::Matrix<double, kMaxNumCss, 3>& H,
                            const Eigen::Vector<double, kMaxNumCss>& weights,
                            const Eigen::Vector<double, kMaxNumCss>& y,
                            Eigen::Vector3d& x);

    /*! Compute the post-fit residuals for the WLS estimate.
        @return the per-sensor residuals, zero beyond the configured sensor count
        @param cssMeas The measured values for the CSS sensors
        @param wlsEst  The WLS estimate computed for the CSS measurements
     */
    Eigen::Vector<double, kMaxNumCss> computeWlsResiduals(const Eigen::Vector<double, kMaxNumCss>& cssMeas,
                                                          const Eigen::Vector3d& wlsEst) const;

    /*! [-] Prior normalized sun heading estimate, body frame components. */
    Eigen::Vector3d dOld = Eigen::Vector3d::Zero();

    /*! [-] Flag indicating a prior sun heading estimate is available for the rate difference. */
    uint32_t priorSignalAvailable{};

    /*! [ns] Time of the previous update; zero until the first call after a re-initialization. */
    uint64_t priorTime{};
};

#endif
