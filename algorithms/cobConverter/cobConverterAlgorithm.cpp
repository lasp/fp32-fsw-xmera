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

    // Camera calibration matrix K (Christian Eqs. 6, 16, 18, 21) is derived in cobConverter.rst.
    const float pX = 2.0F * safeTanf(fieldOfViewX / 2.0F);
    const float pY = 2.0F * safeTanf(fieldOfViewY / 2.0F);

    this->dX = resolutionX / pX;
    const float dY = resolutionY / pY;

    // Assume the principal point (up, vp) is at the image center.
    const float up = resolutionX / 2.0F;
    const float vp = resolutionY / 2.0F;

    this->X = 1.0F / this->dX;
    this->Y = 1.0F / dY;

    // Average angular field of view per pixel [rad/pixel]; an average-scale approximation (see rst).
    this->ifov_x = fieldOfViewX / resolutionX;
    this->ifov_y = fieldOfViewY / resolutionY;

    this->cameraCalibrationMatrix << this->dX, alpha, up, 0.0F, dY, vp, 0.0F, 0.0F, 1.0F;

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
 * Depending on the configured method, computes a brightness offset factor @c gamma
 * (Binary) and the sun direction angle @c phi in the image plane.
 * Also sets @c validCom in the returned result, since a correction is always applied when this
 * function is called.
 *
 * @param filterVehPosition Spacecraft position
 * @param vehSunPntBdy Sun-pointing direction
 * @param dcm_BN Body-to-inertial DCM for the current cycle (from computeRotations).
 * @return Phase-angle correction terms (alphaPA, phi, gamma, spacecraftRange, Rc, validCom) for the
 *         current cycle. Only called when a correction method is configured, so validCom is always
 *         true in the result.
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
 * @brief Compute centers of brightness and mass in pixel coordinates.
 * @param cobCenterOfBrightness pixel-based center of brightness.
 * @param gamma Phase-angle offset factor (0 when no correction is configured).
 * @param Rc Object radius in pixels (0 when no correction is configured).
 * @param phi Sun direction in the image plane (0 when no correction is configured).
 * @return Tuple of (centerOfBrightness, centerOfMass) as 3-vectors in homogeneous pixel coords.
 */
std::tuple<Eigen::Vector3f, Eigen::Vector3f> CobConverterAlgorithm::computeCentersOfInterest(
    const Eigen::Vector2f& cobCenterOfBrightness,
    const float gamma,
    const float Rc,
    const float phi) {
    // Center of Brightness in pixel space
    Eigen::Vector3f centerOfBrightness{cobCenterOfBrightness(0), cobCenterOfBrightness(1), 1.0F};

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
 * @param cobPixelsFound detected-pixel count
 * @param filterVehPositionCovariance Filter position covariance
 * @param correction Phase-angle correction terms for the current cycle (from
 *        computePhaseAngleCorrection).
 * @return Total COM/COB covariance in the body frame for the current cycle.
 */
Eigen::Matrix3f CobConverterAlgorithm::computeCameraFrameUncertainty(
    const int32_t& cobPixelsFound,
    const Eigen::Matrix3d& filterVehPositionCovariance,
    const PhaseAngleCorrectionResult& correction) const {
    // Compute partials of the phase angle and Geometric model correction
    const float scaleFactor = safeSqrtf(static_cast<float>(cobPixelsFound) / kSphereSolidAngle);
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
        // deltaAlpha_delta_R omits the 1/sin(alpha) factor from the full d(alpha)/d(r) expression
        // (see cobConverter.rst): it cancels against a matching missing factor in
        // deltaBinary_deltaAlpha, avoiding a division that blows up near alpha = 0 or pi.
        const Eigen::RowVector3d deltaAlpha_delta_R = -(sr * rr);

        const Eigen::RowVector3d deltaBinary_r = deltaBinary_delta_r + (deltaBinary_deltaAlpha * deltaAlpha_delta_R);

        const double total_deltaBinary_partials =
            deltaBinary_r * filterVehPositionCovariance * deltaBinary_r.transpose();
        const float term2 = powf(deltaBinary_delta_R, 2.0F) * powf(this->cfg.getRadiusUncertainty(), 2.0F);
        const double sigma_beta_squared = total_deltaBinary_partials + static_cast<double>(term2);

        // Rotates the 1-D phase-angle variance into a 2-D image-plane covariance and applies the
        // angle->pixel/pixel->NIC scale conversion; see the "corrected equation" derivation in
        // cobConverter.rst.
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
    unitVecOutput.unitVecValid = correction.validCom && goodOutlierCheck;

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
 * @param cob COB measurement payload.
 * @param attitude Vehicle attitude knowledge (body orientation and sun direction).
 * @param filter Filter position state and covariance.
 * @return Populated CobConverterOutput (zeroed if cob.cobValid is false or cob.cobPixelsFound is zero).
 */
CobConverterOutput CobConverterAlgorithm::updateState(const CobMeasurement& cob,
                                                      const VehicleAttitude& attitude,
                                                      const FilterState& filter) const {
    CobConverterOutput output;

    if (cob.cobValid && cob.cobPixelsFound != 0 &&
        filter.filterVehPosition.stableNorm() > static_cast<double>(this->cfg.getRadius())) {
        const Rotations rotations = this->computeRotations(attitude.sigma_BN);

        PhaseAngleCorrectionResult correction;
        if (this->cfg.getPhaseAngleCorrectionMethod() != PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg) {
            correction =
                this->computePhaseAngleCorrection(filter.filterVehPosition, attitude.vehSunPntBdy, rotations.dcm_BN);
        }
        auto [centerOfBrightness, centerOfMass] = CobConverterAlgorithm::computeCentersOfInterest(
            cob.cobCenterOfBrightness, correction.gamma, correction.Rc, correction.phi);
        correction.validCom = centerOfMass.allFinite();
        auto [rhatCOB_C, rhatCOM_C] = this->computeRelevantVectors(centerOfBrightness, centerOfMass);
        const Eigen::Matrix3f covar_B =
            this->computeCameraFrameUncertainty(cob.cobPixelsFound, filter.filterVehPositionCovariance, correction);

        if (rhatCOB_C.allFinite() && rhatCOM_C.allFinite() && covar_B.allFinite()) {
            bool goodOutlierCheck = true;
            if (this->cfg.isOutlierDetectionEnabled()) {
                goodOutlierCheck = this->cobOutlierDetection(
                    filter.filterVehPosition, filter.filterVehPositionCovariance, covar_B, rhatCOB_C, rotations.dcm_NC);
                output.diagnostic.coberrorOutlierTrigger = !goodOutlierCheck;
            }
            CobConverterAlgorithm::populateOutputMessages(cob.cobTimeTag,
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
 * @param filterVehPosition Filter position
 * @param filterVehPositionCovariance Filter position covariance
 * @param covar_B Total COM/COB covariance in the body frame for the current cycle.
 * @param rhatCOB_C COB unit vector in the camera frame for the current cycle.
 * @param dcm_NC Inertial-to-camera DCM for the current cycle.
 * @return True unless the COB error prediction exceeds the sigma-based gate.
 */
bool CobConverterAlgorithm::cobOutlierDetection(const Eigen::Vector3d& filterVehPosition,
                                                const Eigen::Matrix3d& filterVehPositionCovariance,
                                                const Eigen::Matrix3f& covar_B,
                                                const Eigen::Vector3f& rhatCOB_C,
                                                const Eigen::Matrix3f& dcm_NC) const {
    const Eigen::Vector3d& rNav_BN_N = filterVehPosition;
    const Eigen::Vector3f rhatNav_N = rNav_BN_N.stableNormalized().cast<float>();
    const Eigen::Matrix3f covarNav_N = (filterVehPositionCovariance / pow(rNav_BN_N.stableNorm(), 2)).cast<float>();

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
