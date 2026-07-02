#include "cobConverterAlgorithm.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
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
static Eigen::Matrix3f computeTotalCobCovariance(const Eigen::Matrix3f& covarNav_N,
                                                 const Eigen::Matrix3f& covarAtt_B,
                                                 const Eigen::Matrix3f& covarCob_C,
                                                 const Eigen::Matrix3f& dcm_CN,
                                                 const Eigen::Matrix3f& dcm_CB,
                                                 const Eigen::Matrix3f& cameraCalibrationMatrix);

/**
 * @brief Construct a CobConverterAlgorithm.
 * @param method Phase-angle correction method to apply.
 * @param radiusObject Object radius in meters (must be > 0).
 * @throws std::invalid_argument If radiusObject is not > 0.
 */
CobConverterAlgorithm::CobConverterAlgorithm(const PhaseAngleCorrectionMethodAlgorithm method,
                                             const float radiusObject) {
    this->phaseAngleCorrectionMethod = method;
    if (radiusObject <= 0) {
        FSW_THROW_INVALID_ARGUMENT("cobConverter: radiusObject must be > 0");
    }
    this->objectRadius = radiusObject;
}

/** @brief Default destructor. */
CobConverterAlgorithm::~CobConverterAlgorithm() = default;

/**
 * @brief Compute camera calibration matrix and camera in body DCM
 *
 * Uses the camera model and navigation attitude to compute:
 *  - Body->Camera (B->C)
 *  - Camera calibration matrix K and its inverse
 *  - Pixel scale, IFOV, and other camera parameters
 *
 * @param input Camera model specifications.
 */
void CobConverterAlgorithm::computeCameraParameters(const CobConverterInput& input) {
    // apply the mrpToDcm in double precision
    const Eigen::Vector3d bodyToCameraMrpD = input.bodyToCameraMrp.cast<double>();
    this->dcm_CB = mrpToDcm(bodyToCameraMrpD).cast<float>();

    // Camera parameters
    const float alpha = 0.0F;
    const float pX = 2.0F * safeTanf(input.fieldOfView / 2.0F);
    const float pY = 2.0F * safeTanf(input.fieldOfView * input.resolutionY / input.resolutionX / 2.0F);
    this->dX = input.resolutionX / pX;
    const float dY = input.resolutionY / pY;
    const float up = input.resolutionX / 2.0F;
    const float vp = input.resolutionY / 2.0F;
    this->X = 1.0F / this->dX;
    this->Y = 1.0F / dY;
    this->ifov_x = input.fieldOfView / this->dX * pX;
    this->ifov_y = input.fieldOfView / dY * pY;
    this->cameraId = input.cameraId;

    // Build K and K^{-1}
    this->cameraCalibrationMatrix << this->dX, alpha, up, 0.0F, dY, vp, 0.0F, 0.0F, 1.0F;
    this->cameraCalibrationMatrixInverse << 1.0F / this->dX, -alpha / (this->dX * dY),
        (alpha * vp - dY * up) / (this->dX * dY), 0.0F, 1.0F / dY, -vp / dY, 0.0F, 0.0F, 1.0F;
}

/**
 * @brief Compute time varying DCMs
 *
 * Uses the camera model and navigation attitude to compute:
 *  - Body->Inertial (B->N) DCMs
 *  - Inertial->Camera (N->C) DCM
 *
 * @param input Navigation attitude buffer containing MRP sigma_BN.
 */
void CobConverterAlgorithm::computeRotations(const CobConverterInput& input) {
    this->dcm_BN = mrpToDcm(input.sigma_BN);
    this->dcm_NC = this->dcm_BN.transpose() * this->dcm_CB.transpose();
}

/**
 * @brief Compute phase-angle correction term and related angles.
 *
 * Depending on the configured method, computes a brightness offset factor @c gamma
 * (Binary) and the sun direction angle @c phi in the image plane.
 * Also sets @c validCOM when a correction is applied.
 *
 * @param input Spacecraft position (input.filterState) and sun-pointing attitude (input.vehSunPntBdy)
 * @note Computes and stores: @c alphaPA, @c phi, @c gamma, @c spacecraftRange, @c Rc.
 */
void CobConverterAlgorithm::computePhaseAngleCorrection(const CobConverterInput& input) {
    this->sc_position = input.filterVehPosition;
    const Eigen::Vector3f rhat_N = this->sc_position.stableNormalized().cast<float>();
    const Eigen::Vector3f shat_B = input.vehSunPntBdy.stableNormalized();
    this->shat_N = this->dcm_BN.transpose() * shat_B;
    const Eigen::Vector3f shat_C = this->dcm_CB * shat_B;

    this->alphaPA = safeAcosf(rhat_N.transpose() * this->shat_N);  // phase angle
    this->phi = safeAtan2f(shat_C[1], shat_C[0]);                  // sun direction in image plane
    if (this->phaseAngleCorrectionMethod == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg) {
        // Using phase angle correction assuming a binarized image (brightness either 0 or 1)
        const float oneMinusCosAlpha = 2.0F * powf(safeSinf(this->alphaPA / 2.0F), 2.0F);
        this->gamma = kBinaryPhaseCoeff * oneMinusCosAlpha;
    }
    this->spacecraftRange = this->sc_position.stableNorm();
    this->Rc = static_cast<float>(this->objectRadius * this->dX / this->spacecraftRange);  // object radius in pixels
}

/**
 * @brief Compute centers of brightness and mass in pixel coordinates.
 * @param input COB message payload (pixel-based center of brightness).
 * @return Tuple of (centerOfBrightness, centerOfMass) as 3-vectors in homogeneous pixel coords.
 */
std::tuple<Eigen::Vector3f, Eigen::Vector3f> CobConverterAlgorithm::computeCentersOfInterest(
    const CobConverterInput& input) const {
    // Center of Brightness in pixel space
    Eigen::Vector3f centerOfBrightness;
    centerOfBrightness[0] = input.cobCenterOfBrightness[0];
    centerOfBrightness[1] = input.cobCenterOfBrightness[1];
    centerOfBrightness[2] = 1.0F;

    // Center of Mass in pixel space (offset by phase-angle correction)
    Eigen::Vector3f centerOfMass;
    centerOfMass[0] = centerOfBrightness[0] - this->gamma * this->Rc * safeCosf(this->phi);
    centerOfMass[1] = centerOfBrightness[1] - this->gamma * this->Rc * safeSinf(this->phi);
    centerOfMass[2] = 1.0F;
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
    float const x = unCalibratedVector[0];
    float const y = unCalibratedVector[1];
    float const r2 = x * x + y * y;
    float const r4 = r2 * r2;
    float const r6 = r2 * r4;
    float const k1 = this->calibrationCoefficients.k1;
    float const k2 = this->calibrationCoefficients.k2;
    float const k3 = this->calibrationCoefficients.k3;
    float const p1 = this->calibrationCoefficients.p1;
    float const p2 = this->calibrationCoefficients.p2;

    float const kPolynomial = (1 + k1 * r2 + k2 * r4 + k3 * r6);

    calibratedVector[0] = x * kPolynomial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    calibratedVector[1] = y * kPolynomial + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);

    return calibratedVector;
}

/**
 * @brief Compute unit vectors in the camera frame from pixel coordinates.
 * @param centerOfBrightness 3-vector (homogeneous) pixel coordinates of COB.
 * @param centerOfMass 3-vector (homogeneous) pixel coordinates of COM.
 * @note Populates @c rhatCOB_C, @c rhatCOM_C and caches @c rhatCOBNorm.
 */
void CobConverterAlgorithm::computeRelevantVectors(const Eigen::Vector3f& centerOfBrightness,
                                                   const Eigen::Vector3f& centerOfMass) {
    // Retrieve the vector from target to camera and normalize. When all coefficients
    // are zero calibrateDistortions is a no-op, so we always call it.
    this->rhatCOB_C = -this->calibrateDistortions(this->cameraCalibrationMatrixInverse * centerOfBrightness);
    this->rhatCOM_C = -this->calibrateDistortions(this->cameraCalibrationMatrixInverse * centerOfMass);
    this->rhatCOB_C.stableNormalize();
    this->rhatCOM_C.stableNormalize();
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
 */
void CobConverterAlgorithm::computeCameraFrameUncertainty(const CobConverterInput& input) {
    // Compute partials of the phase angle and Geometric model correction
    const float scaleFactor = safeSqrtf(static_cast<float>(input.cobPixelsFound) / kSphereSolidAngle);
    this->covar_B.setZero();
    if (phaseAngleCorrectionMethod == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg &&
        this->objectRadiusUncertainty > 0) {
        const float oneMinusCosAlpha = 2.0F * powf(safeSinf(this->alphaPA / 2.0F), 2.0F);
        const float constants_deltaR = static_cast<float>(
            kBinaryPhaseCoeff * this->objectRadius / this->sc_position.stableNorm() * oneMinusCosAlpha /
            (1.0 +
             pow(kBinaryPhaseCoeff * this->objectRadius / this->sc_position.stableNorm() * oneMinusCosAlpha, 2.0)));

        const Eigen::RowVector3d deltaBinary_delta_r =
            (-this->sc_position.stableNormalized() / this->sc_position.stableNorm() * constants_deltaR);

        const float deltaBinary_delta_R = (constants_deltaR / this->objectRadius);

        const float deltaBinary_deltaAlpha = static_cast<float>(
            kBinaryPhaseCoeff * this->objectRadius / this->sc_position.stableNorm() /
            (1.0 +
             pow(kBinaryPhaseCoeff * this->objectRadius / this->sc_position.stableNorm() * oneMinusCosAlpha, 2.0)));

        const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix3d::Identity();
        const Eigen::RowVector3d sr = this->shat_N.cast<double>() / this->sc_position.stableNorm();
        const Eigen::Matrix<double, 3, 3> rr =
            I - (this->sc_position.stableNormalized() * this->sc_position.stableNormalized().transpose());
        const Eigen::RowVector3d deltaAlpha_delta_R = sr * rr;

        const Eigen::RowVector3d deltaBinary_r = deltaBinary_delta_r + (deltaBinary_deltaAlpha * deltaAlpha_delta_R);

        double total_deltaBinary_partials =
            deltaBinary_r * input.filterVehPositionCovariance * deltaBinary_r.transpose();
        const float term2 = powf(deltaBinary_delta_R, 2.0F) * powf(this->objectRadiusUncertainty, 2.0F);
        double sigma_beta_squared = total_deltaBinary_partials + static_cast<double>(term2);

        // Define diagonal COM covariance in C and rotate to B
        Eigen::Matrix3f covarCom_C = Eigen::Matrix3f::Zero();
        covarCom_C(0, 0) =
            powf(this->X, 2) + static_cast<float>(sigma_beta_squared / powf(this->ifov_x, 2) * safeCosf(this->phi));
        covarCom_C(1, 1) =
            powf(this->Y, 2) + static_cast<float>(sigma_beta_squared / powf(this->ifov_y, 2) * safeSinf(this->phi));
        covarCom_C(2, 2) = 1.0F;
        covarCom_C *= scaleFactor;
        const Eigen::Matrix3f covarCom_B = this->dcm_CB.transpose() * covarCom_C * this->dcm_CB;

        // Add COM covariance in B frame to get total covariance
        this->covar_B = covarCom_B + this->covarAtt_BN_B;

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
        this->covar_B = covarCom_B + this->covarAtt_BN_B;
    }
}

/**
 * @brief Populate a CobConverterOutput with unit-vector and COM fields.
 *
 * @param timeTag Measurement timestamp (nanoseconds).
 * @param centerOfMass COM in homogeneous pixel coordinates.
 * @param centerOfBrightness COB in homogeneous pixel coordinates.
 * @param output CobConverterOutput to fill (coberrorOutlierTrigger is left untouched).
 */
void CobConverterAlgorithm::populateOutputMessages(const uint64_t timeTag,
                                                   const Eigen::Vector3f& centerOfMass,
                                                   const Eigen::Vector3f& centerOfBrightness,
                                                   CobConverterOutput& output) const {
    const Eigen::Vector3f rhatCOM_N = this->dcm_NC * this->rhatCOM_C;
    const Eigen::Vector3f rhatCOM_B = this->dcm_BN * rhatCOM_N;
    output.covar_N = this->dcm_BN.transpose() * this->covar_B * this->dcm_BN;
    output.covar_C = this->dcm_NC.transpose() * output.covar_N * this->dcm_NC;
    output.covar_B = this->covar_B;
    output.rhat_BN_N = rhatCOM_N;
    output.rhat_BN_C = this->rhatCOM_C;
    output.rhat_BN_B = rhatCOM_B;
    output.unitVecTimeTag = static_cast<double>(timeTag) * kNano2Sec;
    output.unitVecValid = (this->validCOM && this->goodOutlierCheck);
    const Eigen::Vector2f centerOfBrightnessXY(centerOfBrightness(0), centerOfBrightness(1));
    output.centerOfBrightness = centerOfBrightnessXY;
    const Eigen::Vector2f centerOfMassXY(centerOfMass(0), centerOfMass(1));
    output.centerOfMass = centerOfMassXY;
    output.offsetFactor = this->gamma;
    output.objectPixelRadius = static_cast<int>(this->Rc);
    output.phaseAngle = this->alphaPA;
    output.sunDirection = this->phi;
    output.cameraID = this->cameraId;
    output.comTimeTag = timeTag;
    output.comValid = this->validCOM;
}

/**
 * @brief Update step: convert pixel-based COB into unit vectors and return all outputs.
 *
 * Computes camera parameters, rotations, optional phase-angle correction,
 * outlier detection, and populates a CobConverterOutput struct.
 *
 * @param input All input message payloads bundled as a CobConverterInput.
 * @return Populated CobConverterOutput (zeroed if input.cobValid is false or input.cobPixelsFound is zero).
 */
CobConverterOutput CobConverterAlgorithm::updateState(const CobConverterInput& input) {
    CobConverterOutput output;

    if (input.cobValid && input.cobPixelsFound != 0) {
        this->computeCameraParameters(input);
        this->computeRotations(input);

        // Phase angle correction
        if (this->phaseAngleCorrectionMethod != PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg) {
            this->validCOM = true;
            this->computePhaseAngleCorrection(input);
        }

        auto [centerOfBrightness, centerOfMass] = this->computeCentersOfInterest(input);
        this->computeRelevantVectors(centerOfBrightness, centerOfMass);
        this->computeCameraFrameUncertainty(input);

        if (this->performOutlierDetection) {
            this->cobOutlierDetection(input, output);
        }

        this->populateOutputMessages(input.cobTimeTag, centerOfMass, centerOfBrightness, output);
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
static Eigen::Matrix3f computeTotalCobCovariance(const Eigen::Matrix3f& covarNav_N,
                                                 const Eigen::Matrix3f& covarAtt_B,
                                                 const Eigen::Matrix3f& covarCob_C,
                                                 const Eigen::Matrix3f& dcm_CN,
                                                 const Eigen::Matrix3f& dcm_CB,
                                                 const Eigen::Matrix3f& cameraCalibrationMatrix) {
    Eigen::Matrix3f covarAtt_C = dcm_CB * covarAtt_B * dcm_CB.transpose();
    Eigen::Matrix3f covarNav_C = dcm_CN * covarNav_N * dcm_CN.transpose();
    Eigen::Matrix3f covarTotal_C = covarCob_C + covarAtt_C + covarNav_C;
    Eigen::Matrix3f covarImage = cameraCalibrationMatrix * covarTotal_C * cameraCalibrationMatrix.transpose();

    return covarImage;
}

/**
 * @brief Perform outlier detection on the COB measurement.
 *
 * Projects the filter's expected unit vector to pixel space and compares against the
 * measured COB. Uses either a specified standard deviation or one derived from the
 * combined image covariance to perform a sigma-based gate.
 *
 * @param input Filter position and position covariance
 * @param output CobConverterOutput whose coberrorOutlierTrigger field is set.
 */
void CobConverterAlgorithm::cobOutlierDetection(const CobConverterInput& input, CobConverterOutput& output) {
    const Eigen::Vector3d rNav_BN_N = input.filterVehPosition;
    const Eigen::Vector3f rhatNav_N = rNav_BN_N.stableNormalized().cast<float>();
    const Eigen::Matrix3f covarNav_N =
        (input.filterVehPositionCovariance / pow(rNav_BN_N.stableNorm(), 2)).cast<float>();

    Eigen::Vector3f rhatCOB_C_znorm =
        -this->rhatCOB_C;  // turn unit vector from asteroid to camera into unit vector from camera to asteroid
    rhatCOB_C_znorm /= rhatCOB_C_znorm[2];  // make z-component 1 for image plane
    const Eigen::Vector3f cob = this->cameraCalibrationMatrix * rhatCOB_C_znorm;

    // assume that the time of the last filter update corresponds to the current timestep (so no propagation required)
    Eigen::Vector3f rhatNav_C = this->dcm_NC.transpose() * (-rhatNav_N);
    rhatNav_C /= rhatNav_C[2];
    const Eigen::Vector3f cobNav = this->cameraCalibrationMatrix * rhatNav_C;

    const float cobErrorPrediction = (cob - cobNav).stableNorm();
    float sigma = 0.0F;
    if (this->specifiedStandardDeviation) {
        sigma = this->standardDeviation;
    } else {
        Eigen::Matrix3f covarImage =
            computeTotalCobCovariance(covarNav_N,
                                      this->covarAtt_BN_B,
                                      this->dcm_CB.transpose() * this->covar_B * this->dcm_CB.transpose(),
                                      this->dcm_NC.transpose(),
                                      this->dcm_CB,
                                      this->cameraCalibrationMatrix);
        sigma = safeSqrtf(std::max(covarImage(0, 0), covarImage(1, 1)));
    }

    if (cobErrorPrediction < this->numStandardDeviations * sigma) {
        this->goodOutlierCheck = true;
        output.coberrorOutlierTrigger = false;
    } else {
        output.coberrorOutlierTrigger = true;
    }
}

/**
 * @brief Set the object radius.
 * @param radius Object radius in meters (must be > 0).
 * @throws std::invalid_argument If radius is not > 0.
 */
void CobConverterAlgorithm::setRadius(const float radius) {
    if (radius <= 0) {
        FSW_THROW_INVALID_ARGUMENT("cobConverter: radius must be > 0");
    }
    this->objectRadius = radius;
}

/**
 * @brief Get the object radius.
 * @return Object radius in meters.
 */
float CobConverterAlgorithm::getRadius() const { return this->objectRadius; }

/**
 * @brief Set the object radius uncertainty.
 * @param radiusUncertainty Object radius uncertainty in meters (>= 0).
 * @throws std::invalid_argument If radiusUncertainty is < 0.
 */
void CobConverterAlgorithm::setRadiusUncertainty(const float radiusUncertainty) {
    if (radiusUncertainty < 0) {
        FSW_THROW_INVALID_ARGUMENT("cobConverter: radiusUncertainty must be >= 0");
    }
    this->objectRadiusUncertainty = radiusUncertainty;
}

/**
 * @brief Get the object radius uncertainty.
 * @return Object radius uncertainty in meters.
 */
float CobConverterAlgorithm::getRadiusUncertainty() const { return this->objectRadiusUncertainty; }

/**
 * @brief Set the attitude error covariance matrix in body frame (for unit vector measurements).
 * @param covAtt_BN_B 3x3 attitude covariance in body frame.
 */
void CobConverterAlgorithm::setAttitudeCovariance(const Eigen::Matrix3f& covAtt_BN_B) {
    this->covarAtt_BN_B = covAtt_BN_B;
}

/**
 * @brief Get the attitude error covariance matrix in body frame (for unit vector measurements).
 * @return 3x3 attitude covariance in body frame.
 */
Eigen::Matrix3f CobConverterAlgorithm::getAttitudeCovariance() const { return this->covarAtt_BN_B; }

/**
 * @brief Set the number of standard deviations for outlier gating.
 * @param num Number of sigmas (> 0).
 * @throws std::invalid_argument If num is not > 0.
 */
void CobConverterAlgorithm::setNumStandardDeviations(const float num) {
    if (num <= 0.0F) {
        FSW_THROW_INVALID_ARGUMENT("cobConverter: numStandardDeviations must be > 0");
    }
    this->numStandardDeviations = num;
}

/**
 * @brief Get the configured number of standard deviations for outlier gating.
 * @return Number of sigmas.
 */
float CobConverterAlgorithm::getNumStandardDeviations() const { return this->numStandardDeviations; }

/**
 * @brief Set an explicit standard deviation for the expected COB error.
 * @param num Standard deviation (> 0).
 * @note When set, outlier detection will use this fixed value instead of deriving one.
 * @throws std::invalid_argument If num is not > 0.
 */
void CobConverterAlgorithm::setStandardDeviation(const float num) {
    if (num <= 0.0F) {
        FSW_THROW_INVALID_ARGUMENT("cobConverter: standardDeviation must be > 0");
    }
    this->standardDeviation = num;
    this->specifiedStandardDeviation = true;
}

/**
 * @brief Get the explicitly specified standard deviation (if set).
 * @return Standard deviation value.
 */
float CobConverterAlgorithm::getStandardDeviation() const { return this->standardDeviation; }

/**
 * @brief Determine whether a standard deviation has been explicitly specified.
 * @return True if specified, false otherwise.
 */
bool CobConverterAlgorithm::isStandardDeviationSpecified() const { return this->specifiedStandardDeviation; }

/**
 * @brief Enable COB outlier detection.
 */
void CobConverterAlgorithm::enableOutlierDetection() { this->performOutlierDetection = true; }

/**
 * @brief Disable COB outlier detection.
 */
void CobConverterAlgorithm::disableOutlierDetection() { this->performOutlierDetection = false; }

/**
 * @brief Check whether COB outlier detection is enabled.
 * @return True if enabled, false otherwise.
 */
bool CobConverterAlgorithm::isOutlierDetectionEnabled() const { return this->performOutlierDetection; }

/**
 * @brief Set the Brown Conrady calibration coefficients
 * @param coefficients CalibrationCoefficients
 */
void CobConverterAlgorithm::setBrownConradyCoefficients(const CalibrationCoefficients& coefficients) {
    this->calibrationCoefficients = coefficients;
}

/**
 * @brief Get the Brown Conrady calibration coefficients.
 * @return CalibrationCoefficients
 */
CalibrationCoefficients CobConverterAlgorithm::getBrownConradyCoefficients() const {
    return this->calibrationCoefficients;
}
