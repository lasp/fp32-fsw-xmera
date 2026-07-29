#include "cobConverterAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <math.h>
#include <numbers>

// Binary phase-angle correction factor (binarized image model)
static constexpr float kBinaryPhaseCoeff = 4.0F / (3.0F * std::numbers::pi_v<float>);
// Full solid angle of a sphere [sr], used in pixel uncertainty scale factor
static constexpr float kSphereSolidAngle = 4.0F * std::numbers::pi_v<float>;

/**
 * @brief Compute total COB covariance in image space given unit-vector covariances.
 *
 * The covariance contributions include navigation, attitude, and COB measurement terms.
 * They are rotated into the camera frame and mapped to pixel space via the camera
 * calibration matrix K.
 *
 * @param covarNav_N Navigation covariance (inertial frame)
 * @param covarAtt_B Attitude covariance (body frame)
 * @param covarCob_C COB covariance (camera frame)
 * @param dcm_CN DCM from camera to inertial (C->N)
 * @param dcm_CB DCM from camera to body (C->B)
 * @param cameraCalibrationMatrix Camera calibration matrix K
 * @return Image-space covariance matrix in pixel units
 */
namespace {
Eigen::Matrix3f computeTotalCobCovariance(const Eigen::Matrix3f& covarNav_N,
                                          const Eigen::Matrix3f& covarAtt_B,
                                          const Eigen::Matrix3f& covarCob_C,
                                          const Eigen::Matrix3f& dcm_CN,
                                          const Eigen::Matrix3f& dcm_CB,
                                          const Eigen::Matrix3f& cameraCalibrationMatrix);
}  // namespace

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

    // Christian Eq. (6) defines the normalized image-plane coordinates as
    //
    //     x = tan(beta_H),    y = tan(beta_V).
    //
    // Therefore, the full normalized image-plane spans associated with
    // symmetric angular intervals [-FOV/2, FOV/2] are
    //
    //     pX = x_max - x_min = 2 tan(fovX/2),
    //     pY = y_max - y_min = 2 tan(fovY/2).
    const float pX = 2.0F * safeTanf(fieldOfViewX / 2.0F);
    const float pY = 2.0F * safeTanf(fieldOfViewY / 2.0F);

    // Christian Eq. (16) maps normalized image-plane coordinates to pixels:
    //
    //     u = dX * x + up,
    //     v = dY * y + vp.
    //
    // Let xLeft and xRight be the normalized coordinates of the horizontal
    // image boundaries. Subtracting Eq. (16) at the two boundaries gives
    //
    //     uRight - uLeft = dX * (xRight - xLeft).
    //
    // Here, pX = xRight - xLeft is the full normalized image-plane width,
    // and the corresponding pixel-space width is modeled as resolutionX.
    // Therefore,
    //
    //     dX = resolutionX / pX.
    //
    // Applying the same argument vertically, with
    //
    //     pY = yBottom - yTop,
    //     vBottom - vTop = resolutionY,
    //
    // gives
    //
    //     dY = resolutionY / pY.
    //
    // Thus, dX and dY are scale factors in pixels per unit normalized
    // image-plane coordinate.
    this->dX = resolutionX / pX;
    const float dY = resolutionY / pY;

    // Assume that the principal point (up, vp) is at the image center.
    const float up = resolutionX / 2.0F;
    const float vp = resolutionY / 2.0F;

    // From the inverse of Christian Eq. (16), one pixel corresponds to
    // 1/dX and 1/dY in normalized image-plane coordinates.
    this->X = 1.0F / this->dX;
    this->Y = 1.0F / dY;

    // Average angular field of view per pixel [rad/pixel].
    //
    // Since dX * pX = resolutionX and dY * pY = resolutionY,
    //
    //     ifov_x = fovX / resolutionX = fovX / (dX * pX),
    //     ifov_y = fovY / resolutionY = fovY / (dY * pY).
    //
    // These are average angular pixel scales; Christian's exact tangent
    // projection produces a slightly position-dependent local angular scale.
    this->ifov_x = fieldOfViewX / resolutionX;
    this->ifov_y = fieldOfViewY / resolutionY;

    // Construct the camera calibration matrix K from Christian Eq. (18):
    //
    //         [ dX  alpha  up ]
    //     K = [  0   dY    vp ].
    //         [  0    0     1 ]
    this->cameraCalibrationMatrix << this->dX, alpha, up, 0.0F, dY, vp, 0.0F, 0.0F, 1.0F;

    // Construct K^{-1} using Christian Eq. (21).
    this->cameraCalibrationMatrixInverse << 1.0F / this->dX, -alpha / (this->dX * dY),
        ((alpha * vp) - (dY * up)) / (this->dX * dY), 0.0F, 1.0F / dY, -vp / dY, 0.0F, 0.0F, 1.0F;
}

/**
 * @brief Compute time varying DCMs
 *
 * Uses the camera model and navigation attitude to compute:
 *  - Body->Inertial (B->N) DCMs
 *  - Inertial->Camera (N->C) DCM
 *
 * @param input Navigation attitude buffer containing MRP sigma_BN.
 * @return Rotations dcm_BN (B->N) and dcm_NC (N->C) for the current cycle.
 */
Rotations CobConverterAlgorithm::computeRotations(const CobConverterInput& input) const {
    Rotations rotations;
    rotations.dcm_BN = mrpToDcm(input.sigma_BN);
    rotations.dcm_NC = rotations.dcm_BN.transpose() * this->dcm_CB.transpose();
    return rotations;
}

/**
 * @brief Compute phase-angle correction term and related angles.
 *
 * Depending on the configured method, computes a brightness offset factor @c gamma
 * (Binary) and the sun direction angle @c phi in the image plane.
 * Also sets @c validCom in the returned result, since a correction is always applied when this
 * function is called.
 *
 * @param input Spacecraft position (input.filterState) and sun-pointing attitude (input.vehSunPntBdy)
 * @param dcm_BN Body-to-inertial DCM for the current cycle (from computeRotations).
 * @return Phase-angle correction terms (alphaPA, phi, gamma, spacecraftRange, Rc, validCom) for the
 *         current cycle. Only called when a correction method is configured, so validCom is always
 *         true in the result.
 */
PhaseAngleCorrectionResult CobConverterAlgorithm::computePhaseAngleCorrection(const CobConverterInput& input,
                                                                              const Eigen::Matrix3f& dcm_BN) const {
    PhaseAngleCorrectionResult correction;
    correction.validCom = true;
    correction.sc_position = input.filterVehPosition;
    const Eigen::Vector3f rhat_N = correction.sc_position.stableNormalized().cast<float>();
    const Eigen::Vector3f shat_B = input.vehSunPntBdy.stableNormalized();
    correction.shat_N = dcm_BN.transpose() * shat_B;
    const Eigen::Vector3f shat_C = this->dcm_CB * shat_B;

    correction.alphaPA = safeAcosf(rhat_N.transpose() * correction.shat_N);  // phase angle
    correction.phi = safeAtan2f(shat_C(1), shat_C(0));                       // sun direction in image plane
    if (this->cfg.getPhaseAngleCorrectionMethod() == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg) {
        // Using phase angle correction assuming a binarized image (brightness either 0 or 1)
        const float oneMinusCosAlpha = 2.0F * powf(safeSinf(correction.alphaPA / 2.0F), 2.0F);
        correction.gamma = kBinaryPhaseCoeff * oneMinusCosAlpha;
    }
    correction.spacecraftRange = correction.sc_position.stableNorm();
    correction.Rc =
        static_cast<float>(this->cfg.getRadius() * this->dX / correction.spacecraftRange);  // object radius in pixels
    return correction;
}

/**
 * @brief Compute centers of brightness and mass in pixel coordinates.
 * @param input COB message payload (pixel-based center of brightness).
 * @param gamma Phase-angle offset factor (0 when no correction is configured).
 * @param Rc Object radius in pixels (0 when no correction is configured).
 * @param phi Sun direction in the image plane (0 when no correction is configured).
 * @return Tuple of (centerOfBrightness, centerOfMass) as 3-vectors in homogeneous pixel coords.
 */
std::tuple<Eigen::Vector3f, Eigen::Vector3f> CobConverterAlgorithm::computeCentersOfInterest(
    const CobConverterInput& input,
    const float gamma,
    const float Rc,
    const float phi) {
    // Center of Brightness in pixel space
    Eigen::Vector3f centerOfBrightness{input.cobCenterOfBrightness(0), input.cobCenterOfBrightness(1), 1.0F};

    // Center of Mass in pixel space (offset by phase-angle correction)
    Eigen::Vector3f const centerOfMass{centerOfBrightness(0) - (gamma * Rc * safeCosf(phi)),
                                       centerOfBrightness(1) - (gamma * Rc * safeSinf(phi)),
                                       1.0F};
    return {centerOfBrightness, centerOfMass};
}

/**
 * @brief Apply the Brown-Conrady distortion model to a normalized image-plane coordinate.
 *
 * Uses the stored radial (k1, k2, k3) and tangential (p1, p2) coefficients. With all
 * coefficients zero, the input is returned unchanged.
 *
 * @param unCalibratedVector Normalized image-plane coordinate in homogeneous form (z = 1).
 * @return Corrected coordinate in the same homogeneous form.
 */
Eigen::Vector3f CobConverterAlgorithm::calibrateDistortions(const Eigen::Vector3f& unCalibratedVector) const {
    Eigen::Vector3f calibratedVector{0.0F, 0.0F, 1.0F};
    float const x = unCalibratedVector(0);
    float const y = unCalibratedVector(1);
    float const r2 = (x * x) + (y * y);
    float const r4 = r2 * r2;
    float const r6 = r2 * r4;
    const CalibrationCoefficients calibrationCoefficients = this->cfg.getCalibrationCoefficients();
    float const k1 = calibrationCoefficients.k1;
    float const k2 = calibrationCoefficients.k2;
    float const k3 = calibrationCoefficients.k3;
    float const p1 = calibrationCoefficients.p1;
    float const p2 = calibrationCoefficients.p2;

    float const kPolynomial = (1 + (k1 * r2) + (k2 * r4) + (k3 * r6));

    calibratedVector(0) = (x * kPolynomial) + (2 * p1 * x * y) + (p2 * (r2 + (2 * x * x)));
    calibratedVector(1) = (y * kPolynomial) + (2 * p2 * x * y) + (p1 * (r2 + (2 * y * y)));

    return calibratedVector;
}

/**
 * @brief Compute unit vectors in the camera frame from pixel coordinates.
 * @param centerOfBrightness 3-vector (homogeneous) pixel coordinates of COB.
 * @param centerOfMass 3-vector (homogeneous) pixel coordinates of COM.
 * @return Tuple of (rhatCOB_C, rhatCOM_C).
 */
std::tuple<Eigen::Vector3f, Eigen::Vector3f> CobConverterAlgorithm::computeRelevantVectors(
    const Eigen::Vector3f& centerOfBrightness,
    const Eigen::Vector3f& centerOfMass) const {
    // Retrieve the vector from target to camera and normalize. When all coefficients
    // are zero calibrateDistortions is a no-op, so we always call it.
    Eigen::Vector3f rhatCOB_C = -this->calibrateDistortions(this->cameraCalibrationMatrixInverse * centerOfBrightness);
    Eigen::Vector3f rhatCOM_C = -this->calibrateDistortions(this->cameraCalibrationMatrixInverse * centerOfMass);
    rhatCOB_C.stableNormalize();
    rhatCOM_C.stableNormalize();
    return {rhatCOB_C, rhatCOM_C};
}

/**
 * @brief Compute the measurement uncertainty in the camera frame.
 *
 * If Binary phase-angle correction is used and a nonzero object radius uncertainty
 * is provided, incorporates the propagated uncertainty of the phase-angle correction.
 * Otherwise, uses a diagonal COB covariance scaled by the number of pixels found,
 * then rotates it into the body frame and adds attitude covariance.
 *
 * @param input Filter position covariance and detected-pixel count.
 * @param correction Phase-angle correction terms for the current cycle (from
 *        computePhaseAngleCorrection).
 * @return Total COM/COB covariance in the body frame for the current cycle.
 */
Eigen::Matrix3f CobConverterAlgorithm::computeCameraFrameUncertainty(
    const CobConverterInput& input,
    const PhaseAngleCorrectionResult& correction) const {
    // Compute partials of the phase angle and Geometric model correction
    const float scaleFactor = safeSqrtf(static_cast<float>(input.cobPixelsFound) / kSphereSolidAngle);
    Eigen::Matrix3f covar_B = Eigen::Matrix3f::Zero();
    const float radius = this->cfg.getRadius();
    if (this->cfg.getPhaseAngleCorrectionMethod() == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg &&
        this->cfg.getRadiusUncertainty() > 0) {
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
        // deltaAlpha_delta_R is d(alpha)/d(r). The full expression is -s_hat^T/(r*sin(alpha)) *
        // (I - r_hat*r_hat^T) (see cobConverter.rst), but deltaBinary_deltaAlpha above is missing
        // the matching sin(alpha) factor from d(betaG)/d(alpha) -- since the two are only ever
        // used together as a product below, sin(alpha) cancels exactly, and the leading minus
        // sign is the only piece that actually needs to be applied here. Folding in the full
        // 1/sin(alpha) term instead would introduce a real division that blows up as alpha
        // approaches 0 or pi, for no benefit once the cancellation is accounted for.
        const Eigen::RowVector3d deltaAlpha_delta_R = -(sr * rr);

        const Eigen::RowVector3d deltaBinary_r = deltaBinary_delta_r + (deltaBinary_deltaAlpha * deltaAlpha_delta_R);

        const double total_deltaBinary_partials =
            deltaBinary_r * input.filterVehPositionCovariance * deltaBinary_r.transpose();
        const float term2 = powf(deltaBinary_delta_R, 2.0F) * powf(this->cfg.getRadiusUncertainty(), 2.0F);
        const double sigma_beta_squared = total_deltaBinary_partials + static_cast<double>(term2);

        // The phase-angle correction moves COM away from COB along the 1-D direction
        // (cos(phi), sin(phi)) in the image plane, so sigma_beta_squared is the variance of a
        // single scalar magnitude along that line (zero variance perpendicular to it). Turning
        // that 1-D angular variance into a 2-D covariance aligned along phi is a similarity
        // transform by the rotation matrix R(phi) = [cos(phi) -sin(phi); sin(phi) cos(phi)]:
        //
        //     Cov_angle = R(phi) * diag(sigma_beta_squared, 0) * R(phi)^T
        //               = sigma_beta_squared * [cos(phi)^2         cos(phi)*sin(phi);
        //                                       cos(phi)*sin(phi)  sin(phi)^2       ]
        //
        // which is PSD for any phi (a similarity transform of a non-negative diagonal matrix).
        // Converting from angular units (rad^2) to normalized image-plane units takes two
        // separate, anisotropic conversions the rest of this file already keeps distinct:
        // angle -> pixels via ifov_x/ifov_y (rad/px, an average-scale approximation -- see the
        // comment in computeCameraParameters), then pixels -> normalized image-plane coordinates
        // via X/Y (the exact, tan-based per-axis pixel scale used for the baseline term below).
        // These two conversions are NOT interchangeable in general -- X/ifov_x deviates from 1
        // by ~26% at the wide end of the supported FOV range -- so both steps are needed, applied
        // as the diagonal congruence transform D * Cov_angle * D with D = diag(X/ifov_x, Y/ifov_y).
        // A congruence transform by a real matrix preserves PSD-ness, and adding the baseline COB
        // pixel-noise diagonal diag(X^2, Y^2) keeps the sum PSD, since the sum of two PSD matrices
        // is PSD.
        const float cosPhi = safeCosf(correction.phi);
        const float sinPhi = safeSinf(correction.phi);
        const float directionX = this->X / this->ifov_x;
        const float directionY = this->Y / this->ifov_y;
        const double correctionXX = sigma_beta_squared * static_cast<double>(directionX * directionX * cosPhi * cosPhi);
        const double correctionYY = sigma_beta_squared * static_cast<double>(directionY * directionY * sinPhi * sinPhi);
        const double correctionXY = sigma_beta_squared * static_cast<double>(directionX * directionY * cosPhi * sinPhi);

        // Define COM covariance in C (now with the off-diagonal cross term) and rotate to B
        Eigen::Matrix3f covarCom_C = Eigen::Matrix3f::Zero();
        covarCom_C(0, 0) = powf(this->X, 2) + static_cast<float>(correctionXX);
        covarCom_C(1, 1) = powf(this->Y, 2) + static_cast<float>(correctionYY);
        covarCom_C(0, 1) = static_cast<float>(correctionXY);
        covarCom_C(1, 0) = static_cast<float>(correctionXY);
        covarCom_C(2, 2) = 1.0F;
        covarCom_C *= scaleFactor;
        const Eigen::Matrix3f covarCom_B = this->dcm_CB.transpose() * covarCom_C * this->dcm_CB;

        // Add COM covariance in B frame to get total covariance
        covar_B = covarCom_B + this->cfg.getAttitudeCovariance();

    } else {
        // Define diagonal COB covariance
        Eigen::Matrix3f covarCob_C;
        covarCob_C.setZero();
        covarCob_C(0, 0) = powf(this->X, 2);
        covarCob_C(1, 1) = powf(this->Y, 2);
        covarCob_C(2, 2) = 1.0F;
        // Scale by number of pixels and rotate to B
        covarCob_C *= scaleFactor;
        const Eigen::Matrix3f covarCom_B = this->dcm_CB.transpose() * covarCob_C * this->dcm_CB;
        covar_B = covarCom_B + this->cfg.getAttitudeCovariance();
    }
    return covar_B;
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
 * @param covar_B Total COM/COB covariance in the body frame.
 * @param goodOutlierCheck True unless outlier detection is enabled and flagged this cycle.
 * @param unitVecOutput Unit-vector output to fill.
 * @param comOutput COM output to fill.
 */
void CobConverterAlgorithm::populateOutputMessages(
    const uint64_t timeTag,
    const Eigen::Vector3f& centerOfMass,  // NOLINT(bugprone-easily-swappable-parameters)
    const Eigen::Vector3f& centerOfBrightness,
    const Rotations& rotations,
    const PhaseAngleCorrectionResult& correction,
    const Eigen::Vector3f& rhatCOM_C,
    const Eigen::Matrix3f& covar_B,
    const bool goodOutlierCheck,
    CobConverterUnitVecOutput& unitVecOutput,
    CobConverterComOutput& comOutput) {
    const Eigen::Vector3f rhatCOM_N = rotations.dcm_NC * rhatCOM_C;
    const Eigen::Vector3f rhatCOM_B = rotations.dcm_BN * rhatCOM_N;
    unitVecOutput.covar_N = rotations.dcm_BN.transpose() * covar_B * rotations.dcm_BN;
    unitVecOutput.covar_C = rotations.dcm_NC.transpose() * unitVecOutput.covar_N * rotations.dcm_NC;
    unitVecOutput.covar_B = covar_B;
    unitVecOutput.rhat_BN_N = rhatCOM_N;
    unitVecOutput.rhat_BN_C = rhatCOM_C;
    unitVecOutput.rhat_BN_B = rhatCOM_B;
    unitVecOutput.unitVecTimeTag = static_cast<double>(timeTag) * kNano2Sec;
    unitVecOutput.unitVecValid = (correction.validCom && goodOutlierCheck);
    const Eigen::Vector2f centerOfBrightnessXY(centerOfBrightness(0), centerOfBrightness(1));
    comOutput.centerOfBrightness = centerOfBrightnessXY;
    const Eigen::Vector2f centerOfMassXY(centerOfMass(0), centerOfMass(1));
    comOutput.centerOfMass = centerOfMassXY;
    comOutput.offsetFactor = correction.gamma;
    comOutput.objectPixelRadius = static_cast<int>(correction.Rc);
    comOutput.phaseAngle = correction.alphaPA;
    comOutput.sunDirection = correction.phi;
    comOutput.comTimeTag = timeTag;
    comOutput.comValid = correction.validCom;
}

/**
 * @brief Update step: convert pixel-based COB into unit vectors and return all outputs.
 *
 * Computes rotations, optional phase-angle correction, outlier detection, and populates a
 * CobConverterOutput struct. Camera parameters are precomputed by setConfig() and are not
 * recomputed per cycle.
 *
 * @param input All input message payloads bundled as a CobConverterInput.
 * @return Populated CobConverterOutput (zeroed if input.cobValid is false or input.cobPixelsFound is zero).
 */
CobConverterOutput CobConverterAlgorithm::updateState(const CobConverterInput& input) const {
    CobConverterOutput output;

    if (input.cobValid && input.cobPixelsFound != 0) {
        const Rotations rotations = this->computeRotations(input);

        // Phase angle correction. PhaseAngleCorrectionResult default-constructs to all-zero/false
        // (no correction applied) when NoCorrectionAlg is configured, so this cycle's values never
        // leak stale data from a previous cycle where BinaryAlg happened to be configured -- see
        // the commit message for the staleness bug this replaced.
        PhaseAngleCorrectionResult correction;
        if (this->cfg.getPhaseAngleCorrectionMethod() != PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg) {
            correction = this->computePhaseAngleCorrection(input, rotations.dcm_BN);
        }

        auto [centerOfBrightness, centerOfMass] =
            CobConverterAlgorithm::computeCentersOfInterest(input, correction.gamma, correction.Rc, correction.phi);
        auto [rhatCOB_C, rhatCOM_C] = this->computeRelevantVectors(centerOfBrightness, centerOfMass);
        const Eigen::Matrix3f covar_B = this->computeCameraFrameUncertainty(input, correction);

        bool goodOutlierCheck = true;
        if (this->cfg.isOutlierDetectionEnabled()) {
            goodOutlierCheck = this->cobOutlierDetection(input, covar_B, rhatCOB_C, rotations.dcm_NC);
            output.diagnostic.coberrorOutlierTrigger = !goodOutlierCheck;
        }

        CobConverterAlgorithm::populateOutputMessages(input.cobTimeTag,
                                                      centerOfMass,
                                                      centerOfBrightness,
                                                      rotations,
                                                      correction,
                                                      rhatCOM_C,
                                                      covar_B,
                                                      goodOutlierCheck,
                                                      output.unitVec,
                                                      output.com);
    }

    return output;
}

/**
 * @brief Helper to combine nav, attitude, and COB covariances and map to image space.
 *
 * @param covarNav_N Navigation covariance (inertial frame).
 * @param covarAtt_B Attitude covariance (body frame).
 * @param covarCob_C COB covariance (camera frame).
 * @param dcm_CN DCM camera-to-inertial.
 * @param dcm_CB DCM camera-to-body.
 * @param cameraCalibrationMatrix Camera calibration matrix K.
 * @return Image-space covariance (pixels).
 */
namespace {
Eigen::Matrix3f computeTotalCobCovariance(
    const Eigen::Matrix3f& covarNav_N,
    const Eigen::Matrix3f& covarAtt_B,  // NOLINT(bugprone-easily-swappable-parameters)
    const Eigen::Matrix3f& covarCob_C,
    const Eigen::Matrix3f& dcm_CN,
    const Eigen::Matrix3f& dcm_CB,
    const Eigen::Matrix3f& cameraCalibrationMatrix) {
    const Eigen::Matrix3f covarAtt_C = dcm_CB * covarAtt_B * dcm_CB.transpose();
    const Eigen::Matrix3f covarNav_C = dcm_CN * covarNav_N * dcm_CN.transpose();
    const Eigen::Matrix3f covarTotal_C = covarCob_C + covarAtt_C + covarNav_C;
    Eigen::Matrix3f covarImage = cameraCalibrationMatrix * covarTotal_C * cameraCalibrationMatrix.transpose();

    return covarImage;
}
}  // namespace

/**
 * @brief Perform outlier detection on the COB measurement.
 *
 * Projects the filter's expected unit vector to pixel space and compares against the
 * measured COB. Uses either a specified standard deviation or one derived from the
 * combined image covariance to perform a sigma-based gate.
 *
 * @param input Filter position and position covariance
 * @param covar_B Total COM/COB covariance in the body frame for the current cycle.
 * @param rhatCOB_C COB unit vector in the camera frame for the current cycle.
 * @param dcm_NC Inertial-to-camera DCM for the current cycle.
 * @return True unless the COB error prediction exceeds the sigma-based gate.
 */
bool CobConverterAlgorithm::cobOutlierDetection(const CobConverterInput& input,
                                                const Eigen::Matrix3f& covar_B,
                                                const Eigen::Vector3f& rhatCOB_C,
                                                const Eigen::Matrix3f& dcm_NC) const {
    const Eigen::Vector3d rNav_BN_N = input.filterVehPosition;
    const Eigen::Vector3f rhatNav_N = rNav_BN_N.stableNormalized().cast<float>();
    const Eigen::Matrix3f covarNav_N =
        (input.filterVehPositionCovariance / pow(rNav_BN_N.stableNorm(), 2)).cast<float>();

    Eigen::Vector3f rhatCOB_C_znorm =
        -rhatCOB_C;  // turn unit vector from asteroid to camera into unit vector from camera to asteroid
    rhatCOB_C_znorm /= rhatCOB_C_znorm(2);  // make z-component 1 for image plane
    const Eigen::Vector3f cob = this->cameraCalibrationMatrix * rhatCOB_C_znorm;

    // assume that the time of the last filter update corresponds to the current timestep (so no propagation required)
    Eigen::Vector3f rhatNav_C = dcm_NC.transpose() * (-rhatNav_N);
    rhatNav_C /= rhatNav_C(2);
    const Eigen::Vector3f cobNav = this->cameraCalibrationMatrix * rhatNav_C;

    const float cobErrorPrediction = (cob - cobNav).stableNorm();
    float sigma = 0.0F;
    if (this->cfg.isStandardDeviationSpecified()) {
        sigma = this->cfg.getStandardDeviation();
    } else {
        Eigen::Matrix3f covarImage =
            computeTotalCobCovariance(covarNav_N,
                                      this->cfg.getAttitudeCovariance(),
                                      this->dcm_CB.transpose() * covar_B * this->dcm_CB.transpose(),
                                      dcm_NC.transpose(),
                                      this->dcm_CB,
                                      this->cameraCalibrationMatrix);
        sigma = safeSqrtf(std::max(covarImage(0, 0), covarImage(1, 1)));
    }
    return cobErrorPrediction < this->cfg.getNumStandardDeviations() * sigma;
}
