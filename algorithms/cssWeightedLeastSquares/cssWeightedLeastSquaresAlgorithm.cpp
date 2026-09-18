#include "cssWeightedLeastSquaresAlgorithm.h"

#include "utilities/fsw/safeMath.h"

#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <optional>

/*! Smallest physically meaningful CSS reading. A coarse sun sensor cannot report a negative cosine,
    so the predicted measurement is floored here before differencing against the observation. */
static constexpr float kMinCssMeasurement = 0.0F;

/*! Largest CSS reading the estimator uses. A cosine cannot exceed one; the margin allows for the
    calibration and the noise on a sensor that points at the sun. */
static constexpr float kMaxCssMeasurement = 1.1F;

/*! Relative tolerance for treating a normal matrix as singular, sized at a few multiples of the
    working precision's machine epsilon. The determinant of an n-by-n matrix scales as the n-th power
    of the matrix norm, so the absolute threshold handed to Eigen is this factor times the norm
    raised to the matrix dimension, which keeps the test scale invariant. */
static constexpr float kSingularDeterminantRelativeTolerance = 1e-6F;

/*! Number of active measurements below which the fit is exactly determined and the measurement
    weights carry no information. */
static constexpr uint32_t kMinMeasurementsForWeightedFit = 3;

/*! Smallest cross product magnitude between two successive headings that still fixes a rotation axis.
    The magnitude is the sine of the angle between them, so below this the headings are parallel or
    antiparallel to within the working precision and the direction the cross product reports is
    round-off rather than rotation. */
static constexpr float kMinRotationAxisMagnitude = 1e-6F;

namespace {

/*! Invert a normal matrix, rejecting it when its determinant is singular at the matrix's own scale.
    @return the inverse, or nothing when the matrix is singular
    @param matrix the normal matrix to invert
 */
template <typename MatrixT>
std::optional<MatrixT> invertNormalMatrix(const MatrixT& matrix) {
    const float norm = matrix.reshaped().stableNorm();
    float threshold = kSingularDeterminantRelativeTolerance;
    for (int dimension = 0; dimension < MatrixT::RowsAtCompileTime; ++dimension) {
        threshold *= norm;
    }

    MatrixT inverse = MatrixT::Zero();
    float determinant = 0.0F;
    bool invertible = false;
    matrix.computeInverseAndDetWithCheck(inverse, determinant, invertible, threshold);

    std::optional<MatrixT> result;
    if (invertible) {
        result = inverse;
    }
    return result;
}

}  // namespace

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
    this->priorSunHeading_B.setZero();
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame, along with the inertial angular velocity
 derived from two successive sun heading estimates
 @return the estimator products for this cycle
 @param cosValues [-] Per-sensor cosine readings, indexed by sensor
 */
CssWeightedLeastSquaresOutput CssWeightedLeastSquaresAlgorithm::update(
    const Eigen::Vector<float, kMaxNumCssSensors>& cosValues) {
    /* The predicted pointing vector for each measurement, compacted to the active sensors */
    Eigen::Matrix<float, kMaxNumCssSensors, 3> H = Eigen::Matrix<float, kMaxNumCssSensors, 3>::Zero();
    /* Measurements, compacted to the active sensors */
    Eigen::Vector<float, kMaxNumCssSensors> y = Eigen::Vector<float, kMaxNumCssSensors>::Zero();
    /* The sensor index behind each observation, in observation order */
    std::array<Eigen::Index, kMaxNumCssSensors> activeSensors{};
    uint32_t numCssViewingSun = 0;
    std::optional<Eigen::Vector3f> fit; /* the least squares solution; empty when there is none */
    Eigen::Vector3f sunHeading_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Vector<float, kMaxNumCssSensors> postFitResiduals = Eigen::Vector<float, kMaxNumCssSensors>::Zero();

    for (uint32_t i = 0; i < kMaxNumCssSensors; i = i + 1) {
        /* The upper bound also removes a reading that is not a number, because every comparison with
           one is false. */
        if (this->cfg.getCssAvailability().at(i) == fsw::DeviceAvailability::Available &&
            cosValues(i) > this->cfg.getSensorUseThresh() && cosValues(i) <= kMaxCssMeasurement) {
            H.row(numCssViewingSun) = this->cfg.getCssNHat_B().row(i);
            y(numCssViewingSun) = cosValues(i);
            activeSensors.at(numCssViewingSun) = i;
            numCssViewingSun = numCssViewingSun + 1;
        }
    }

    if (numCssViewingSun > 0) {
        Eigen::Vector<float, kMaxNumCssSensors> weights = Eigen::Vector<float, kMaxNumCssSensors>::Ones();
        if (this->cfg.getUseWeights()) {
            weights = y;
        }
        fit = computeWlsmn(numCssViewingSun, weights, H, y);
    }

    if (fit) {
        sunHeading_B = fit->stableNormalized();

        if (this->priorSignalAvailable) {
            const Eigen::Vector3f rotationAxis = sunHeading_B.cross(this->priorSunHeading_B);
            const float rotationAxisMagnitude = rotationAxis.stableNorm();
            /* Leave the rate at zero when the two headings fix no axis to rotate about. Taking the
               angle from the cross product as well as the dot product keeps the significant digits of a
               small angle, which the dot product alone loses because its cosine rounds to one. */
            if (rotationAxisMagnitude > kMinRotationAxisMagnitude) {
                const float principalAngle =
                    safeAtan2f(rotationAxisMagnitude, sunHeading_B.dot(this->priorSunHeading_B));
                omega_BN_B = rotationAxis * (principalAngle / (rotationAxisMagnitude * this->cfg.getControlPeriod()));
            }
        } else {
            this->priorSignalAvailable = true;
        }
        this->priorSunHeading_B = sunHeading_B;
    }

    /* Residuals are measured against the unnormalized fit, which is zero when there was no sun */
    postFitResiduals =
        this->computeWlsResiduals(cosValues, fit.value_or(Eigen::Vector3f::Zero()), activeSensors, numCssViewingSun);

    /* With no sun, or a singular fit, there is no prior heading to difference against */
    if (!fit) {
        this->priorSignalAvailable = false;
    }

    return {.sunHeading_B = sunHeading_B,
            .omega_BN_B = omega_BN_B,
            .postFitResiduals = postFitResiduals,
            .numCssViewingSun = numCssViewingSun};
}

/*! This method computes the post-fit residuals for the WLS estimate. The residuals are indexed by
    observation rather than by sensor slot, so the leading numCssViewingSun entries carry the sensors that
    contributed to the fit and the remainder stay zero.
    @return the residuals of the active sensors, packed into the leading numCssViewingSun entries
    @param cssMeas The measured values for the CSS sensors
    @param wlsEst The WLS estimate computed for the CSS measurements
    @param activeSensors The sensor index behind each observation, in observation order
    @param numCssViewingSun The count on input measurements
*/
Eigen::Vector<float, kMaxNumCssSensors> CssWeightedLeastSquaresAlgorithm::computeWlsResiduals(
    const Eigen::Vector<float, kMaxNumCssSensors>& cssMeas,
    const Eigen::Vector3f& wlsEst,
    const std::array<Eigen::Index, kMaxNumCssSensors>& activeSensors,
    const uint32_t numCssViewingSun) const {
    Eigen::Vector<float, kMaxNumCssSensors> cssResiduals = Eigen::Vector<float, kMaxNumCssSensors>::Zero();

    for (uint32_t observation = 0; observation < numCssViewingSun; observation++) {
        const Eigen::Index sensor = activeSensors.at(observation);
        const float rawDotProduct = wlsEst.dot(this->cfg.getCssNHat_B().row(sensor).transpose());
        /* A coarse sun sensor cannot report a negative cosine, so floor the prediction */
        const float cssDotProduct = rawDotProduct > kMinCssMeasurement ? rawDotProduct : kMinCssMeasurement;
        cssResiduals(observation) = cssMeas(sensor) - cssDotProduct;
    }

    return cssResiduals;
}

/*! This method computes a least squares fit with the given parameters.
 @return the fit, or nothing when the normal matrix is singular
 @param numCssViewingSun The count on input measurements
 @param weights The diagonal of the measurement weighting matrix; only applied when more than two
        measurements are available, as the one- and two-measurement fits are exactly determined
 @param H The predicted pointing vector for each measurement, one per row
 @param y the observation vector for the valid sensors
 */
std::optional<Eigen::Vector3f> CssWeightedLeastSquaresAlgorithm::computeWlsmn(
    const uint32_t numCssViewingSun,
    const Eigen::Vector<float, kMaxNumCssSensors>& weights,
    const Eigen::Matrix<float, kMaxNumCssSensors, 3>& H,
    const Eigen::Vector<float, kMaxNumCssSensors>& y) {
    std::optional<Eigen::Vector3f> fit;

    if (numCssViewingSun == 1) {
        /* The minimum norm solution of the single observation equation, which is the one-measurement
           case of the two-measurement branch below. The row is a unit boresight, so it needs no scaling. */
        fit = Eigen::Vector3f{H.row(0).transpose() * y(0)};
    } else if (numCssViewingSun == 2) {
        /* Two measurements leave the system underdetermined, so take the minimum norm solution */
        const Eigen::Matrix<float, 2, 3> h = H.topRows<2>();
        const std::optional<Eigen::Matrix2f> hhtInverse = invertNormalMatrix(Eigen::Matrix2f{h * h.transpose()});
        if (hhtInverse) {
            fit = Eigen::Vector3f{h.transpose() * *hhtInverse * y.head<2>()};
        }
    } else if (numCssViewingSun >= kMinMeasurementsForWeightedFit) {
        /* The rows of H and the entries of y past
           numCssViewingSun are zero, so the products over the full operands equal the products over the
           active measurements alone. Forming them at full size keeps every intermediate a
           fixed-size Eigen type; a dynamically sized one would allocate, and this build forbids
           heap allocation. */
        const Eigen::Matrix<float, kMaxNumCssSensors, 3> wh = weights.asDiagonal() * H;
        const std::optional<Eigen::Matrix3f> htwhInverse = invertNormalMatrix(Eigen::Matrix3f{H.transpose() * wh});
        if (htwhInverse) {
            fit = Eigen::Vector3f{*htwhInverse * (wh.transpose() * y)};
        }
    }

    return fit;
}
