#ifndef F32XMERA_COB_CONVERTER_ALGORITHM_H
#define F32XMERA_COB_CONVERTER_ALGORITHM_H

#include <Eigen/Dense>
#include <numbers>

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

/**
 * @brief Camera calibration to pinhole Brown-Conrady coefficients
 */
struct CalibrationCoefficients {
    float k1 = 0.0F;
    float k2 = 0.0F;
    float k3 = 0.0F;
    float p1 = 0.0F;
    float p2 = 0.0F;
};

/**
 * @enum PhaseAngleCorrectionMethodAlgorithm
 * @brief Phase-angle correction models for converting COB to COM.
 */
enum class PhaseAngleCorrectionMethodAlgorithm : std::uint8_t { NoCorrectionAlg, BinaryAlg };

/*! Structure containing all COB converter algorithm inputs. */
struct CobConverterInput {
    // COB measurement
    bool cobValid{};                                                  //!< [--] validity flag
    int32_t cobPixelsFound{};                                         //!< [--] bright pixels
    Eigen::Vector2f cobCenterOfBrightness = Eigen::Vector2f::Zero();  //!< [px] COB pixel coords
    uint64_t cobTimeTag{};                                            //!< [ns] measurement time
    // navigation attitude
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();  //!< [--] body-to-inertial MRP
    // sun attitude
    Eigen::Vector3f vehSunPntBdy = Eigen::Vector3f::Zero();  //!< [--] sun direction, body frame
    // filter state: only position (and its covariance) is consumed by this algorithm.
    // The upstream filter state is a fixed 6-d [position (3), velocity (3)]; velocity is unused.
    Eigen::Vector3d filterVehPosition = Eigen::Vector3d::Zero();  //!< [m] spacecraft position, inertial frame
    Eigen::Matrix3d filterVehPositionCovariance =
        Eigen::Matrix3d::Zero();  //!< [m^2] spacecraft position covariance, inertial frame
};

/*! Heading measurement output: unit vector and its covariance (uncertainty) in multiple
    frames. Maps 1:1 onto OpNavUnitVecMsgF32Payload. */
struct CobConverterUnitVecOutput {
    Eigen::Matrix3f covar_N = Eigen::Matrix3f::Zero();    //!< [--] COM covariance, inertial frame
    Eigen::Matrix3f covar_C = Eigen::Matrix3f::Zero();    //!< [--] COM covariance, camera frame
    Eigen::Matrix3f covar_B = Eigen::Matrix3f::Zero();    //!< [--] COM covariance, body frame
    Eigen::Vector3f rhat_BN_N = Eigen::Vector3f::Zero();  //!< [--] COM unit vector, inertial frame
    Eigen::Vector3f rhat_BN_C = Eigen::Vector3f::Zero();  //!< [--] COM unit vector, camera frame
    Eigen::Vector3f rhat_BN_B = Eigen::Vector3f::Zero();  //!< [--] COM unit vector, body frame
    double unitVecTimeTag{};                              //!< [s]  measurement timestamp
    bool unitVecValid{};                                  //!< [--] COM unit vector validity flag
};

/*! Center-of-mass measurement output: COM/COB pixel locations and phase-angle offset
    metadata. Maps 1:1 onto OpNavCOMMsgF32Payload. */
struct CobConverterComOutput {
    Eigen::Vector2f centerOfBrightness = Eigen::Vector2f::Zero();  //!< [px] COB pixel coordinates
    Eigen::Vector2f centerOfMass = Eigen::Vector2f::Zero();        //!< [px] COM pixel coordinates
    float offsetFactor{};                                          //!< [--] phase-angle offset factor (gamma)
    int objectPixelRadius{};                                       //!< [px] object radius in pixels
    float phaseAngle{};                                            //!< [rad] phase angle alpha_PA
    float sunDirection{};                                          //!< [rad] sun direction phi in image plane
    uint64_t comTimeTag{};                                         //!< [ns] measurement timestamp
    bool comValid{};                                               //!< [--] COM validity flag
};

/*! Diagnostic output. Maps 1:1 onto CobConverterDiagnosticMsgF32Payload. */
struct CobConverterDiagnosticOutput {
    bool coberrorOutlierTrigger{};  //!< [--] true if COB error exceeded outlier threshold
};

/*! Structure containing all COB converter algorithm outputs. */
struct CobConverterOutput {
    CobConverterUnitVecOutput unitVec;
    CobConverterComOutput com;
    CobConverterDiagnosticOutput diagnostic;
};

/**
 * @class CobConverterConfig
 * @brief Validated configuration parameters for CobConverterAlgorithm.
 */
class CobConverterConfig final {
   public:
    static CobConverterConfig create(PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod,
                                     float radius,
                                     float radiusUncertainty,
                                     const Eigen::Matrix3f& attitudeCovariance,
                                     float numStandardDeviations,
                                     float standardDeviation,
                                     bool specifiedStandardDeviation,
                                     bool outlierDetectionEnabled,
                                     const CalibrationCoefficients& calibrationCoefficients,
                                     int cameraId,
                                     float fieldOfViewX,
                                     float fieldOfViewY,
                                     float resolutionX,
                                     float resolutionY,
                                     const Eigen::Vector3f& bodyToCameraMrp) {
        if (!isValidPhaseAngleCorrectionMethod(phaseAngleCorrectionMethod)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: phaseAngleCorrectionMethod must be NoCorrectionAlg or BinaryAlg");
        }
        if (!isValidRadius(radius)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: radius must be > 0");
        }
        if (!isValidRadiusUncertainty(radiusUncertainty)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: radiusUncertainty must be >= 0");
        }
        if (!isValidAttitudeCovariance(attitudeCovariance)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: attitudeCovariance must be finite");
        }
        if (!isValidNumStandardDeviations(numStandardDeviations)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: numStandardDeviations must be > 0");
        }
        if (!isValidStandardDeviation(standardDeviation, specifiedStandardDeviation)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: standardDeviation must be > 0 when specified");
        }
        if (!isValidCalibrationCoefficients(calibrationCoefficients)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: calibrationCoefficients must be finite");
        }
        if (!isValidFieldOfView(fieldOfViewX)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: fieldOfViewX must be > 0 and < pi");
        }
        if (!isValidFieldOfView(fieldOfViewY)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: fieldOfViewY must be > 0 and < pi");
        }
        if (!isValidResolutionX(resolutionX)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: resolutionX must be > 0");
        }
        if (!isValidResolutionY(resolutionY)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: resolutionY must be > 0");
        }
        if (!isValidCameraParam(fieldOfViewX, fieldOfViewY)) {
            FSW_THROW_INVALID_ARGUMENT(
                "cobConverter: fieldOfViewX/fieldOfViewY combination pushes the camera "
                "model's internal tan() argument into the safeTanf clamp zone near +/-pi/2");
        }
        if (!isValidBodyToCameraMrp(bodyToCameraMrp)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: bodyToCameraMrp must be finite");
        }
        return {phaseAngleCorrectionMethod,
                radius,
                radiusUncertainty,
                attitudeCovariance,
                numStandardDeviations,
                standardDeviation,
                specifiedStandardDeviation,
                outlierDetectionEnabled,
                calibrationCoefficients,
                cameraId,
                fieldOfViewX,
                fieldOfViewY,
                resolutionX,
                resolutionY,
                bodyToCameraMrp};
    }

    static bool isValidPhaseAngleCorrectionMethod(PhaseAngleCorrectionMethodAlgorithm method) {
        return method == PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg ||
               method == PhaseAngleCorrectionMethodAlgorithm::BinaryAlg;
    }
    static bool isValidRadius(float radius) { return fsw::is_finite(radius) && radius > 0.0F; }
    static bool isValidRadiusUncertainty(float radiusUncertainty) {
        return fsw::is_finite(radiusUncertainty) && radiusUncertainty >= 0.0F;
    }
    static bool isValidAttitudeCovariance(const Eigen::Matrix3f& attitudeCovariance) {
        return attitudeCovariance.allFinite();
    }
    static bool isValidNumStandardDeviations(float numStandardDeviations) {
        return fsw::is_finite(numStandardDeviations) && numStandardDeviations > 0.0F;
    }
    static bool isValidStandardDeviation(float standardDeviation, bool specifiedStandardDeviation) {
        return !specifiedStandardDeviation || (fsw::is_finite(standardDeviation) && standardDeviation > 0.0F);
    }
    // No isValidOutlierDetectionEnabled — any bool value is valid.
    static bool isValidCalibrationCoefficients(const CalibrationCoefficients& coefficients) {
        return fsw::is_finite(coefficients.k1) && fsw::is_finite(coefficients.k2) && fsw::is_finite(coefficients.k3) &&
               fsw::is_finite(coefficients.p1) && fsw::is_finite(coefficients.p2);
    }
    // No isValidCameraId — any int value is valid (camera identifier, no numeric constraint).
    static bool isValidFieldOfView(float fieldOfView) {
        return fsw::is_finite(fieldOfView) && fieldOfView > 0.0F && fieldOfView < std::numbers::pi_v<float>;
    }
    static bool isValidResolutionX(float resolutionX) { return fsw::is_finite(resolutionX) && resolutionX > 0.0F; }
    static bool isValidResolutionY(float resolutionY) { return fsw::is_finite(resolutionY) && resolutionY > 0.0F; }
    static bool isValidBodyToCameraMrp(const Eigen::Vector3f& bodyToCameraMrp) { return bodyToCameraMrp.allFinite(); }
    // Rejects fieldOfViewX/fieldOfViewY values whose safeTanf() argument (pX's is fieldOfViewX/2,
    // pY's is fieldOfViewY/2) comes within kMinPoleDistance of +/-pi/2, since dX/dY inherit tan's
    // ~1/d^2 blowup there and amplify ordinary fp32 rounding error into large errors.
    static bool isValidCameraParam(float fieldOfViewX, float fieldOfViewY) {
        constexpr float kMinPoleDistance = 0.017453F;  // [rad], ~1.0 deg away from the tan() singularity
        constexpr float halfPi = std::numbers::pi_v<float> / 2.0F;
        const float argTanX = fieldOfViewX / 2.0F;
        if (argTanX < -halfPi + kMinPoleDistance || argTanX > halfPi - kMinPoleDistance) {
            return false;
        }
        const float argTanY = fieldOfViewY / 2.0F;
        return argTanY >= -halfPi + kMinPoleDistance && argTanY <= halfPi - kMinPoleDistance;
    }

    PhaseAngleCorrectionMethodAlgorithm getPhaseAngleCorrectionMethod() const { return phaseAngleCorrectionMethod; }
    float getRadius() const { return radius; }
    float getRadiusUncertainty() const { return radiusUncertainty; }
    Eigen::Matrix3f getAttitudeCovariance() const { return attitudeCovariance; }
    float getNumStandardDeviations() const { return numStandardDeviations; }
    float getStandardDeviation() const { return standardDeviation; }
    bool isStandardDeviationSpecified() const { return specifiedStandardDeviation; }
    bool isOutlierDetectionEnabled() const { return outlierDetectionEnabled; }
    CalibrationCoefficients getCalibrationCoefficients() const { return calibrationCoefficients; }
    int getCameraId() const { return cameraId; }
    float getFieldOfViewX() const { return fieldOfViewX; }
    float getFieldOfViewY() const { return fieldOfViewY; }
    float getResolutionX() const { return resolutionX; }
    float getResolutionY() const { return resolutionY; }
    Eigen::Vector3f getBodyToCameraMrp() const { return bodyToCameraMrp; }

   private:
    CobConverterConfig(PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod,
                       float radius,
                       float radiusUncertainty,
                       const Eigen::Matrix3f& attitudeCovariance,
                       float numStandardDeviations,
                       float standardDeviation,
                       bool specifiedStandardDeviation,
                       bool outlierDetectionEnabled,
                       const CalibrationCoefficients& calibrationCoefficients,
                       int cameraId,
                       float fieldOfViewX,
                       float fieldOfViewY,
                       float resolutionX,
                       float resolutionY,
                       const Eigen::Vector3f& bodyToCameraMrp)
        : phaseAngleCorrectionMethod(phaseAngleCorrectionMethod),
          radius(radius),
          radiusUncertainty(radiusUncertainty),
          attitudeCovariance(attitudeCovariance),
          numStandardDeviations(numStandardDeviations),
          standardDeviation(standardDeviation),
          specifiedStandardDeviation(specifiedStandardDeviation),
          outlierDetectionEnabled(outlierDetectionEnabled),
          calibrationCoefficients(calibrationCoefficients),
          cameraId(cameraId),
          fieldOfViewX(fieldOfViewX),
          fieldOfViewY(fieldOfViewY),
          resolutionX(resolutionX),
          resolutionY(resolutionY),
          bodyToCameraMrp(bodyToCameraMrp) {}

    PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod;
    float radius;
    float radiusUncertainty;
    Eigen::Matrix3f attitudeCovariance;
    float numStandardDeviations;
    float standardDeviation;
    bool specifiedStandardDeviation;
    bool outlierDetectionEnabled;
    CalibrationCoefficients calibrationCoefficients;
    int cameraId;
    float fieldOfViewX;
    float fieldOfViewY;
    float resolutionX;
    float resolutionY;
    Eigen::Vector3f bodyToCameraMrp;
};

/*! Body-to-inertial and inertial-to-camera rotations for the current cycle, derived from the
    current attitude input. dcm_CB is config-derived (from bodyToCameraMrp) and cached separately
    on the algorithm, since it doesn't depend on per-cycle input. */
struct Rotations {
    Eigen::Matrix3f dcm_BN = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f dcm_NC = Eigen::Matrix3f::Zero();
};

/*! Phase-angle correction terms for the current cycle. Default-constructed (all zero/false) when
    phaseAngleCorrectionMethod is NoCorrectionAlg, so no correction is applied to COM. */
struct PhaseAngleCorrectionResult {
    Eigen::Vector3d sc_position = Eigen::Vector3d::Zero();
    double spacecraftRange = 0.0;
    Eigen::Vector3f shat_N = Eigen::Vector3f::Zero();
    float alphaPA = 0.0F;
    float phi = 0.0F;
    float gamma = 0.0F;
    float Rc = 0.0F;
    bool validCom = false;
};

/**
 * @class CobConverterAlgorithm
 * @brief Converts center-of-brightness (COB) pixel measurements into unit vectors
 *        (camera, body, inertial frames), with optional phase-angle correction
 *        and outlier detection.
 */
class CobConverterAlgorithm final {
   public:
    explicit CobConverterAlgorithm(const CobConverterConfig& config);

    void setConfig(const CobConverterConfig& config);
    CobConverterOutput updateState(const CobConverterInput& input) const;
    int getCameraId() const { return this->cfg.getCameraId(); }

   private:
    bool cobOutlierDetection(const Eigen::Vector3d& filterVehPosition,
                             const Eigen::Matrix3d& filterVehPositionCovariance,
                             const Eigen::Matrix3f& covar_B,
                             const Eigen::Vector3f& rhatCOB_C,
                             const Eigen::Matrix3f& dcm_NC) const;
    void computeCameraParameters();
    Rotations computeRotations(const Eigen::Vector3f& sigma_BN) const;
    PhaseAngleCorrectionResult computePhaseAngleCorrection(const Eigen::Vector3d& filterVehPosition,
                                                           const Eigen::Vector3f& vehSunPntBdy,
                                                           const Eigen::Matrix3f& dcm_BN) const;
    static std::tuple<Eigen::Vector3f, Eigen::Vector3f>
    computeCentersOfInterest(const Eigen::Vector2f& cobCenterOfBrightness, float gamma, float Rc, float phi);
    std::tuple<Eigen::Vector3f, Eigen::Vector3f> computeRelevantVectors(const Eigen::Vector3f& centerOfBrightness,
                                                                        const Eigen::Vector3f& centerOfMass) const;
    Eigen::Matrix3f computeCameraFrameUncertainty(const int32_t& cobPixelsFound,
                                                  const Eigen::Matrix3d& filterVehPositionCovariance,
                                                  const PhaseAngleCorrectionResult& correction) const;
    Eigen::Vector3f calibrateDistortions(const Eigen::Vector3f& unCalibratedVector) const;
    static void populateOutputMessages(uint64_t timeTag,
                                       const Eigen::Vector3f& centerOfMass,
                                       const Eigen::Vector3f& centerOfBrightness,
                                       const Rotations& rotations,
                                       const PhaseAngleCorrectionResult& correction,
                                       const Eigen::Vector3f& rhatCOM_C,
                                       const Eigen::Matrix3f& covar_B,
                                       bool goodOutlierCheck,
                                       CobConverterUnitVecOutput& unitVecOutput,
                                       CobConverterComOutput& comOutput);

    CobConverterConfig cfg;
    Eigen::Matrix3f dcm_CB = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrixInverse = Eigen::Matrix3f::Zero();
    float dX{};
    float X{};
    float Y{};
    float ifov_x{};
    float ifov_y{};
};

#endif  // F32XMERA_COB_CONVERTER_ALGORITHM_H
