#include "cobConverterAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <math.h>
#include <limits>
#include <numbers>

// Binary phase-angle correction factor (binarized image model)
static constexpr float kBinaryPhaseCoeff = 4.0F / (3.0F * std::numbers::pi_v<float>);
// Full solid angle of a sphere [sr], used in pixel uncertainty scale factor
static constexpr float kSphereSolidAngle = 4.0F * std::numbers::pi_v<float>;

/**
 * @brief Construct a CobConverterAlgorithm.
 * @param config Validated configuration parameters.
 */
CobConverterAlgorithm::CobConverterAlgorithm(const CobConverterConfig& config) : cfg(config) { setConfig(config); }

/**
 * @brief Replace the algorithm's configuration.
 * @param config Validated configuration parameters.
 */
void CobConverterAlgorithm::setConfig(const CobConverterConfig& config) {
    this->cfg = config;
    this->computeCameraParameters();
}

/**
 * @brief Compute camera calibration matrix and camera in body DCM
 *
 * Uses the camera model and navigation attitude to compute:
 *  - Body->Camera (B->C)
 *  - Camera calibration matrix K and its inverse
 *  - Pixel scale, IFOV, and other camera parameters
 *
 * Uses the camera model stored in the current config. Invoked from setConfig() (and thus from
 * the constructor, which delegates to it) whenever the config changes; not called per update
 * cycle since these values are invariant between config changes.
 */
void CobConverterAlgorithm::computeCameraParameters() {
    // apply the mrpToDcm in double precision
    const Eigen::Vector3d bodyToCameraMrpD = this->cfg.getBodyToCameraMrp().cast<double>();
    this->dcm_CB = mrpToDcm(bodyToCameraMrpD).cast<float>();

    // Camera parameters
    constexpr float alpha = 0.0F;
    const float fieldOfViewX = this->cfg.getFieldOfViewX();  // Full horizontal angular field of view [rad].
    const float fieldOfViewY = this->cfg.getFieldOfViewY();  // Full vertical angular field of view [rad].

    // Number of pixel columns and rows in the digital image.
    const float resolutionX = this->cfg.getResolutionX();
    const float resolutionY = this->cfg.getResolutionY();

    // Camera calibration matrix K (Christian Eqs. 6, 16, 18, 21) is derived in cobConverter.rst.
    const float pX = 2.0F * safeTanf(fieldOfViewX / 2.0F);
    const float pY = 2.0F * safeTanf(fieldOfViewY / 2.0F);

    this->dX = resolutionX / pX;
    this->dY = resolutionY / pY;

    // Assume the principal point (up, vp) is at the image center.
    const float up = resolutionX / 2.0F;
    const float vp = resolutionY / 2.0F;

    this->X = 1.0F / this->dX;
    this->Y = 1.0F / this->dY;

    // Average angular field of view per pixel [rad/pixel]; an average-scale approximation (see rst).
    this->ifov_x = fieldOfViewX / resolutionX;
    this->ifov_y = fieldOfViewY / resolutionY;

    this->cameraCalibrationMatrix << this->dX, alpha, up, 0.0F, this->dY, vp, 0.0F, 0.0F, 1.0F;

    this->cameraCalibrationMatrixInverse << 1.0F / this->dX, -alpha / (this->dX * this->dY),
        ((alpha * vp) - (this->dY * up)) / (this->dX * this->dY), 0.0F, 1.0F / this->dY, -vp / this->dY, 0.0F, 0.0F,
        1.0F;
}

/**
 * @brief Compute time varying DCMs
 *
 * Uses the camera model and navigation attitude to compute:
 *  - Body->Inertial (B->N) DCMs
 *  - Inertial->Camera (N->C) DCM
 *
 * @param sigma_BN Navigation attitude MRP.
 * @return Rotations dcm_BN (B->N) and dcm_NC (N->C) for the current cycle.
 */
Rotations CobConverterAlgorithm::computeRotations(const Eigen::Vector3f& sigma_BN) const {
    Rotations rotations;
    rotations.dcm_BN = mrpToDcm(sigma_BN);
    rotations.dcm_NC = rotations.dcm_BN.transpose() * this->dcm_CB.transpose();
    return rotations;
}

/**
 * @brief Compute phase-angle correction term and related angles.
 *
 * Computes the Binary offset factor @c gamma and the sun direction @c phi in the image plane.
 *
 * @param filterVehPosition Spacecraft position
 * @param vehSunPntBdy Sun-pointing direction
 * @param dcm_BN Body-to-inertial DCM for the current cycle (from computeRotations).
 * @return alphaPA, phi, gamma, spacecraftRange and Rc. @c validCom is left at its default;
 *         updateState sets it.
 */
PhaseAngleCorrectionResult CobConverterAlgorithm::computePhaseAngleCorrection(const Eigen::Vector3d& filterVehPosition,
                                                                              const Eigen::Vector3f& vehSunPntBdy,
                                                                              const Eigen::Matrix3f& dcm_BN) const {
    PhaseAngleCorrectionResult correction;
    correction.sc_position = filterVehPosition;
    const Eigen::Vector3f rhat_N = correction.sc_position.stableNormalized().cast<float>();
    const Eigen::Vector3f shat_B = vehSunPntBdy.stableNormalized();
    correction.shat_N = dcm_BN.transpose() * shat_B;
    const Eigen::Vector3f shat_C = this->dcm_CB * shat_B;

    correction.alphaPA = safeAcosf(rhat_N.transpose() * correction.shat_N);  // phase angle
    correction.phi = safeAtan2f(shat_C(1), shat_C(0));                       // sun direction in image plane
    const float oneMinusCosAlpha = 2.0F * powf(safeSinf(correction.alphaPA / 2.0F), 2.0F);
    correction.gamma = kBinaryPhaseCoeff * oneMinusCosAlpha;
    correction.spacecraftRange = correction.sc_position.stableNorm();
    correction.Rc =
        static_cast<float>(this->cfg.getRadius() * this->dX / correction.spacecraftRange);  // object radius in pixels
    return correction;
}

/**
 * @brief Invert the Brown-Conrady model by fixed-point iteration.
 *
 * Starting from x_u = x_d, iterates x_u <- (x_d - dx_t) / L (same for y), with
 * L = 1 + k1 r^2 + k2 r^4 + k3 r^6 and tangential terms dx_t, dy_t at the current iterate,
 * until the forward model D(x_u) = x_u * L + d_t reproduces x_d within a tolerance relative to
 * its magnitude, or the iteration limit is reached.
 *
 * @param xDistorted Distorted normalized x.
 * @param yDistorted Distorted normalized y.
 * @param coefficients Brown-Conrady coefficients.
 * @return Undistorted coordinate and fixed-point updates taken; valid is false (with the last
 *         finite iterate) if not converged.
 */
UndistortedCoordinate CobConverterAlgorithm::undistortNormalizedCoordinate(
    const float xDistorted,
    const float yDistorted,
    const CalibrationCoefficients& coefficients) {
    constexpr int kMaxIterations = 50;
    constexpr float kResidualTolerance = 1e-6F;  // [-] relative to the forward-model magnitude

    float const k1 = coefficients.k1;
    float const k2 = coefficients.k2;
    float const k3 = coefficients.k3;
    float const p1 = coefficients.p1;
    float const p2 = coefficients.p2;

    if (!fsw::is_finite(xDistorted) || !fsw::is_finite(yDistorted)) {
        return {xDistorted, yDistorted, false, 0};
    }

    float xUndistorted = xDistorted;
    float yUndistorted = yDistorted;
    for (int iteration = 0; iteration <= kMaxIterations; ++iteration) {
        float const r2 = (xUndistorted * xUndistorted) + (yUndistorted * yUndistorted);
        float const r4 = r2 * r2;
        float const r6 = r2 * r4;
        float const kPolynomial = 1.0F + (k1 * r2) + (k2 * r4) + (k3 * r6);
        float const deltaXt =
            (2.0F * p1 * xUndistorted * yUndistorted) + (p2 * (r2 + (2.0F * xUndistorted * xUndistorted)));
        float const deltaYt =
            (p1 * (r2 + (2.0F * yUndistorted * yUndistorted))) + (2.0F * p2 * xUndistorted * yUndistorted);

        // Converged when the forward model reproduces x_d, relative to the magnitude of its terms
        // (the scale must be finite, or any residual would pass).
        float const residualX = (xUndistorted * kPolynomial) + deltaXt - xDistorted;
        float const residualY = (yUndistorted * kPolynomial) + deltaYt - yDistorted;
        float const residualScale = fmaxf(fmaxf(1.0F, fmaxf(fabsf(xDistorted), fabsf(yDistorted))),
                                          fmaxf(fabsf(xUndistorted * kPolynomial), fabsf(yUndistorted * kPolynomial)));
        if (fsw::is_finite(residualScale) &&
            fmaxf(fabsf(residualX), fabsf(residualY)) <= kResidualTolerance * residualScale) {
            return {xUndistorted, yUndistorted, true, iteration};
        }
        // Iteration limit, or not invertible at L = 0.
        if (iteration == kMaxIterations || fabsf(kPolynomial) < std::numeric_limits<float>::epsilon()) {
            break;
        }

        float const xNext = (xDistorted - deltaXt) / kPolynomial;
        float const yNext = (yDistorted - deltaYt) / kPolynomial;
        if (!fsw::is_finite(xNext) || !fsw::is_finite(yNext)) {
            break;
        }
        xUndistorted = xNext;
        yUndistorted = yNext;
    }
    // Not converged: return the last finite iterate, flagged.
    return {xUndistorted, yUndistorted, false, kMaxIterations};
}

/**
 * @brief Variance of the phase-angle offset beta from the position and radius uncertainties.
 *
 * @param filterVehPositionCovariance [m^2] filter position covariance, inertial frame.
 * @param correction Phase-angle correction terms for the current cycle (from
 *        computePhaseAngleCorrection).
 * @return [rad^2] beta variance.
 */
float CobConverterAlgorithm::computeBetaVar(const Eigen::Matrix3d& filterVehPositionCovariance,
                                            const PhaseAngleCorrectionResult& correction) const {
    // Compute partials of the phase angle and Geometric model correction
    const float radius = this->cfg.getRadius();

    const float oneMinusCosAlpha = 2.0F * powf(safeSinf(correction.alphaPA / 2.0F), 2.0F);
    const auto constants_deltaR = static_cast<float>(
        kBinaryPhaseCoeff * radius / correction.spacecraftRange * oneMinusCosAlpha /
        (1.0 + pow(kBinaryPhaseCoeff * radius / correction.spacecraftRange * oneMinusCosAlpha, 2.0)));

    const Eigen::RowVector3d deltaBinary_delta_r =
        (-correction.sc_position.stableNormalized() / correction.spacecraftRange * constants_deltaR);

    const float deltaBinary_delta_R = (constants_deltaR / radius);

    const auto deltaBinary_deltaAlpha = static_cast<float>(
        kBinaryPhaseCoeff * radius / correction.spacecraftRange /
        (1.0 + pow(kBinaryPhaseCoeff * radius / correction.spacecraftRange * oneMinusCosAlpha, 2.0)));

    const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix3d::Identity();
    const Eigen::RowVector3d sr = correction.shat_N.cast<double>() / correction.spacecraftRange;
    const Eigen::Matrix<double, 3, 3> rr =
        I - (correction.sc_position.stableNormalized() * correction.sc_position.stableNormalized().transpose());
    // deltaAlpha_delta_R omits the 1/sin(alpha) factor from the full d(alpha)/d(r) expression
    // (see cobConverter.rst): it cancels against a matching missing factor in
    // deltaBinary_deltaAlpha, avoiding a division that blows up near alpha = 0 or pi.
    const Eigen::RowVector3d deltaAlpha_delta_R = -(sr * rr);

    const Eigen::RowVector3d deltaBinary_r = deltaBinary_delta_r + (deltaBinary_deltaAlpha * deltaAlpha_delta_R);

    const double total_deltaBinary_partials = deltaBinary_r * filterVehPositionCovariance * deltaBinary_r.transpose();
    // Vanishes for a perfectly known radius; the nav-position partials above still propagate.
    const float term2 = powf(deltaBinary_delta_R, 2.0F) * powf(this->cfg.getRadiusUncertainty(), 2.0F);
    const float sigma_beta_squared = static_cast<float>(total_deltaBinary_partials) + term2;
    return sigma_beta_squared;
}

/**
 * @brief Populate the unit-vector and COM output structs.
 *
 * @param timeTag Measurement timestamp (nanoseconds).
 * @param centerOfMass COM in homogeneous pixel coordinates.
 * @param centerOfBrightness COB in homogeneous pixel coordinates.
 * @param rotations dcm_BN/dcm_NC for the current cycle.
 * @param correction Phase-angle correction terms for the current cycle.
 * @param rhatCOM_C COM unit vector in the camera frame.
 * @param rhatCOB_C COB unit vector in the camera frame.
 * @param output Essential (inertial-frame) output to fill.
 * @param diagnostic Diagnostic output to fill.
 */
void CobConverterAlgorithm::populateOutputMessages(const uint64_t timeTag,
                                                   const Eigen::Vector3f& centerOfMass,
                                                   const Eigen::Vector3f& centerOfBrightness,
                                                   const Rotations& rotations,
                                                   const PhaseAngleCorrectionResult& correction,
                                                   const Eigen::Vector3f& rhatCOM_C,
                                                   const Eigen::Vector3f& rhatCOB_C,
                                                   CobConverterOutput& output,
                                                   CobConverterDiagnosticOutput& diagnostic) {
    const Eigen::Vector3f rhatCOM_N = rotations.dcm_NC * rhatCOM_C;
    output.unitVecTimeTag = static_cast<double>(timeTag) * kNano2Sec;

    diagnostic.rhat_BN_C = rhatCOM_C;
    diagnostic.rhat_BN_B = rotations.dcm_BN * rhatCOM_N;
    diagnostic.rhat_COB_C = rhatCOB_C;
    diagnostic.rhat_COB_N = rotations.dcm_NC * rhatCOB_C;

    const Eigen::Vector2f centerOfBrightnessXY(centerOfBrightness(0), centerOfBrightness(1));
    diagnostic.centerOfBrightness = centerOfBrightnessXY;
    const Eigen::Vector2f centerOfMassXY(centerOfMass(0), centerOfMass(1));
    diagnostic.centerOfMass = centerOfMassXY;
    diagnostic.offsetFactor = correction.gamma;
    diagnostic.objectPixelRadius = static_cast<int>(correction.Rc);
    diagnostic.phaseAngle = correction.alphaPA;
    diagnostic.sunDirection = correction.phi;
    diagnostic.comTimeTag = timeTag;
    diagnostic.comValid = correction.validCom;
}

/**
 * @brief Update step: convert pixel-based COB into unit vectors and return all outputs.
 *
 * Publishes the COM heading and its covariance; the COB heading only feeds outlier detection
 * and the diagnostic. Camera parameters are precomputed by setConfig().
 *
 * @param cob COB measurement payload.
 * @param attitude Vehicle attitude knowledge (body orientation and sun direction).
 * @param filter Filter position state and covariance.
 * @return Populated CobConverterUpdateResult (zeroed if cob.cobValid is false or cob.cobPixelsFound is zero).
 */
CobConverterUpdateResult CobConverterAlgorithm::updateState(const CobMeasurement& cob,
                                                            const VehicleAttitude& attitude,
                                                            const FilterState& filter) const {
    CobConverterUpdateResult result;

    if (cob.cobValid && cob.cobPixelsFound != 0 &&
        filter.filterVehPosition.stableNorm() > static_cast<double>(this->cfg.getRadius())) {
        const Rotations rotations = this->computeRotations(attitude.sigma_BN);

        PhaseAngleCorrectionResult correction =
            this->computePhaseAngleCorrection(filter.filterVehPosition, attitude.vehSunPntBdy, rotations.dcm_BN);

        Eigen::Vector3f rhatCOM_SC_C_Buffer = Eigen::Vector3f::Zero();
        // Eigen::Vector3f rhatCOB_SC_C_Buffer = Eigen::Vector3f::Zero();
        const float uCOB = cob.cobCenterOfBrightness(0);
        const float vCOB = cob.cobCenterOfBrightness(1);
        const float tanBeta = static_cast<float>(this->cfg.getRadius() * correction.gamma / correction.spacecraftRange);
        const float uCOM = uCOB - (tanBeta) * this->dX * safeCosf(correction.phi);
        const float vCOM = vCOB - (tanBeta) * this->dY * safeSinf(correction.phi);
        const Eigen::Vector3f centerOfMass{uCOM, vCOM, 1.0F};
        const Eigen::Vector3f xy1COM = this->cameraCalibrationMatrixInverse * centerOfMass;
        const auto [xCOMCorrected, yCOMCorrected, brownConradyCOMValid, brownConradyCOMIterations] =
            this->undistortNormalizedCoordinate(xy1COM(0), xy1COM(1), this->cfg.getCalibrationCoefficients());
        const Eigen::Vector3f xy1COMCorrected{xCOMCorrected, yCOMCorrected, 1.0F};
        rhatCOM_SC_C_Buffer = -xy1COMCorrected.stableNormalized();
        const Eigen::Vector3f rhatCOM_SC_N_Buffer = rotations.dcm_NC * rhatCOM_SC_C_Buffer;

        const Eigen::Vector3f centerOfBrightness{uCOB, vCOB, 1.0F};
        const Eigen::Vector3f xy1COB = this->cameraCalibrationMatrixInverse * centerOfBrightness;
        const auto [xCOBCorrected, yCOBCorrected, brownConradyCOBValid, brownConradyCOBIterations] =
            this->undistortNormalizedCoordinate(xy1COB(0), xy1COB(1), this->cfg.getCalibrationCoefficients());
        const Eigen::Vector3f xy1COBCorrected{xCOBCorrected, yCOBCorrected, 1.0F};
        result.diagnostic.brownConradyValid =
            brownConradyCOMValid && brownConradyCOBValid;  // depends on both solver validity

        correction.validCom = centerOfMass.allFinite();

        // P_cob_uv
        const Eigen::Matrix2f covarCOBuv =
            (static_cast<float>(cob.cobPixelsFound) / kSphereSolidAngle) * Eigen::Matrix2f::Identity();

        // P_com_xy (distorted ~= corrected)
        const float onePlusTanBetaSq = 1.0f + tanBeta * tanBeta;
        const float sec4Beta = onePlusTanBetaSq * onePlusTanBetaSq;
        const float betaVar = this->computeBetaVar(filter.filterVehPositionCovariance, correction);
        const float phaseVar = betaVar * sec4Beta;
        const float cosPhi = safeCosf(correction.phi);
        const float sinPhi = safeSinf(correction.phi);
        Eigen::Matrix2f S = Eigen::Matrix2f::Zero();
        S(0, 0) = 1.0f / this->dX;
        S(1, 1) = 1.0f / this->dY;
        Eigen::Vector2f a;
        a << cosPhi, sinPhi;
        const Eigen::Matrix2f covarCOMxy = S * covarCOBuv * S.transpose() + phaseVar * (a * a.transpose());

        // P_^C rhat_COM_SC
        const Eigen::Vector3f h{xCOMCorrected, yCOMCorrected, 1.0f};
        const float s = h.stableNorm();
        Eigen::Matrix<float, 3, 2> jacob;
        jacob << yCOMCorrected * yCOMCorrected + 1.0f, -xCOMCorrected * yCOMCorrected, -xCOMCorrected * yCOMCorrected,
            xCOMCorrected * xCOMCorrected + 1.0f, -xCOMCorrected, -yCOMCorrected;
        jacob *= -1.0F / pow(s, 3);
        const Eigen::Matrix3f covarRHatC = jacob * covarCOMxy * jacob.transpose();

        // P_^Nrhat
        const Eigen::Vector3f rHatB = dcm_CB.transpose() * rhatCOM_SC_C_Buffer;
        Eigen::Matrix3f rHatBSkew;
        rHatBSkew << 0.0f, -rHatB(2), rHatB(1), rHatB(2), 0.0f, -rHatB(0), -rHatB(1), rHatB(0), 0.0f;
        const Eigen::Matrix3f covarImageN = rotations.dcm_NC * covarRHatC * rotations.dcm_NC.transpose();
        const Eigen::Matrix3f covarAttitudeN = 16.0f * rotations.dcm_BN.transpose() * rHatBSkew *
                                               this->cfg.getAttitudeCovariance() * rHatBSkew.transpose() *
                                               rotations.dcm_BN;
        const Eigen::Matrix3f covarRHat_N_Buffer = covarImageN + covarAttitudeN;

        if (rhatCOM_SC_C_Buffer.allFinite() && covarRHat_N_Buffer.allFinite()) {
            bool goodOutlierCheck = true;  // passes when outlier detection is disabled
            if (this->cfg.isOutlierDetectionEnabled()) {
                goodOutlierCheck = this->comOutlierDetection(filter.filterVehPosition,
                                                             filter.filterVehPositionCovariance,
                                                             covarRHat_N_Buffer,
                                                             rhatCOM_SC_N_Buffer);
                result.diagnostic.comErrorOutlierTrigger = !goodOutlierCheck;
            }
            CobConverterAlgorithm::populateOutputMessages(cob.cobTimeTag,
                                                          centerOfMass,
                                                          centerOfBrightness,
                                                          rotations,
                                                          correction,
                                                          rhatCOM_SC_C_Buffer,
                                                          -xy1COBCorrected.stableNormalized(),
                                                          result.output,
                                                          result.diagnostic);
            result.output.rhat_BN_N = rhatCOM_SC_N_Buffer;
            result.output.covar_N = covarRHat_N_Buffer;
            result.output.unitVecValid = correction.validCom && goodOutlierCheck;

            result.diagnostic.covar_C = rotations.dcm_NC.transpose() * covarRHat_N_Buffer * rotations.dcm_NC;
            result.diagnostic.covar_B = rotations.dcm_BN * covarRHat_N_Buffer * rotations.dcm_BN.transpose();
        }
    }
    return result;
}

/**
 * @brief Outlier gate: measured COM heading vs filter-predicted heading (both COM->SC, inertial frame).
 *
 * Passes when |rhatCOM_N - rhatNav_N| < numStandardDeviations * sigma, sigma the RMS of that error:
 * standardDeviation * sqrt(1/dX^2 + 1/dY^2) if specified, else sqrt(tr(covar_N + P_nav)), P_nav the filter
 * position covariance projected normal to rhatNav_N over |r|^2.
 *
 * @param filterVehPosition [m] filter spacecraft position relative to the body, inertial frame.
 * @param filterVehPositionCovariance [m^2] filter position covariance, inertial frame.
 * @param covar_N COM heading covariance, inertial frame.
 * @param rhatCOM_N COM->SC unit vector, inertial frame.
 * @return True unless the error exceeds the gate.
 */
bool CobConverterAlgorithm::comOutlierDetection(const Eigen::Vector3d& filterVehPosition,
                                                const Eigen::Matrix3d& filterVehPositionCovariance,
                                                const Eigen::Matrix3f& covar_N,
                                                const Eigen::Vector3f& rhatCOM_N) const {
    const Eigen::Vector3d rhatNav_N = filterVehPosition.stableNormalized();
    const float error = (rhatCOM_N - rhatNav_N.cast<float>()).stableNorm();

    float sigma = 0.0F;
    if (this->cfg.isStandardDeviationSpecified()) {
        sigma = this->cfg.getStandardDeviation() *
                safeSqrtf((1.0F / (this->dX * this->dX)) + (1.0F / (this->dY * this->dY)));
    } else {
        const Eigen::Matrix3d projection = Eigen::Matrix3d::Identity() - (rhatNav_N * rhatNav_N.transpose());
        const Eigen::Matrix3d covarNav_N =
            projection * filterVehPositionCovariance * projection.transpose() / filterVehPosition.squaredNorm();
        sigma = safeSqrtf((covar_N + covarNav_N.cast<float>()).trace());
    }
    return error < this->cfg.getNumStandardDeviations() * sigma;
}
