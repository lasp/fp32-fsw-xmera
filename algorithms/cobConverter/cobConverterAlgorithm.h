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

/*! Brown-Conrady inverse result: undistorted normalized coordinate, solver validity and iterations used. */
struct UndistortedCoordinate {
    float xUndistorted{};  //!< [-] undistorted normalized x
    float yUndistorted{};  //!< [-] undistorted normalized y
    bool valid{};          //!< [--] true if the solver converged to a finite solution
    int iterations{};      //!< [--] Newton iterations taken before returning
};

/*! COB measurement: bright-pixel detection payload. */
struct CobMeasurement {
    bool cobValid{};                                                  //!< [--] validity flag
    int32_t cobPixelsFound{};                                         //!< [--] bright pixels
    Eigen::Vector2f cobCenterOfBrightness = Eigen::Vector2f::Zero();  //!< [px] COB pixel coords
    uint64_t cobTimeTag{};                                            //!< [ns] measurement time
};

/*! Vehicle attitude knowledge: body orientation and sun direction, both body frame. */
struct VehicleAttitude {
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();      //!< [--] body-to-inertial MRP
    Eigen::Vector3f vehSunPntBdy = Eigen::Vector3f::Zero();  //!< [--] sun direction, body frame
};

/*! Filter state: only position (and its covariance) is consumed by this algorithm.
    The upstream filter state is a fixed 6-d [position (3), velocity (3)]; velocity is unused. */
struct FilterState {
    Eigen::Vector3d filterVehPosition = Eigen::Vector3d::Zero();  //!< [m] spacecraft position, inertial frame
    Eigen::Matrix3d filterVehPositionCovariance =
        Eigen::Matrix3d::Zero();  //!< [m^2] spacecraft position covariance, inertial frame
};

/*! Essential heading measurement output: the COM unit vector and its covariance, inertial
    frame only. Maps 1:1 onto OpNavUnitVecMsgF32Payload. */
struct CobConverterOutput {
    Eigen::Matrix3f covar_N = Eigen::Matrix3f::Zero();    //!< [--] COM covariance, inertial frame
    Eigen::Vector3f rhat_BN_N = Eigen::Vector3f::Zero();  //!< [--] COM unit vector, inertial frame
    double unitVecTimeTag{};                              //!< [s]  measurement timestamp
    bool unitVecValid{};                                  //!< [--] COM unit vector validity flag
};

/*! Diagnostic output: the non-inertial frames, the COB quantities and the phase-angle
    correction metadata. Maps 1:1 onto CobConverterDiagnosticMsgF32Payload. */
struct CobConverterDiagnosticOutput {
    Eigen::Matrix3f covar_C = Eigen::Matrix3f::Zero();             //!< [--] COM covariance, camera frame
    Eigen::Matrix3f covar_B = Eigen::Matrix3f::Zero();             //!< [--] COM covariance, body frame
    Eigen::Vector3f rhat_BN_C = Eigen::Vector3f::Zero();           //!< [--] COM unit vector, camera frame
    Eigen::Vector3f rhat_BN_B = Eigen::Vector3f::Zero();           //!< [--] COM unit vector, body frame
    Eigen::Vector3f rhat_COB_C = Eigen::Vector3f::Zero();          //!< [--] COB unit vector, camera frame
    Eigen::Vector3f rhat_COB_N = Eigen::Vector3f::Zero();          //!< [--] COB unit vector, inertial frame
    Eigen::Vector3f rhat_COB_B = Eigen::Vector3f::Zero();          //!< [--] COB unit vector, body frame
    Eigen::Vector2f centerOfBrightness = Eigen::Vector2f::Zero();  //!< [px] COB pixel coordinates
    Eigen::Vector2f centerOfMass = Eigen::Vector2f::Zero();        //!< [px] COM pixel coordinates
    float offsetFactor{};                                          //!< [--] phase-angle offset factor (gamma)
    int objectPixelRadius{};                                       //!< [px] object radius in pixels
    float phaseAngle{};                                            //!< [rad] phase angle alpha_PA
    float sunDirection{};                                          //!< [rad] sun direction phi in image plane
    uint64_t comTimeTag{};                                         //!< [ns] measurement timestamp
    bool comValid{};                                               //!< [--] COM validity flag
    bool comErrorOutlierTrigger{};  //!< [--] true if the COM heading error exceeded the gate
    bool brownConradyCOMValid{};    //!< [--] true if the COM Brown-Conrady undistortion converged
    bool brownConradyCOBValid{};    //!< [--] true if the COB Brown-Conrady undistortion converged
};

/*! Pair returned by updateState: the essential output plus the diagnostic snapshot, so the
    host adapter writes both output messages from one consistent post-update snapshot. */
struct CobConverterUpdateResult {
    CobConverterOutput output;
    CobConverterDiagnosticOutput diagnostic;
};

/**
 * @class CobConverterConfig
 * @brief Validated configuration parameters for CobConverterAlgorithm.
 */
class CobConverterConfig final {
   public:
    static CobConverterConfig create(float radius,
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
        if (!isValidFocalScale(fieldOfViewX, fieldOfViewY, resolutionX, resolutionY)) {
            FSW_THROW_INVALID_ARGUMENT(
                "cobConverter: fieldOfView/resolution combination makes the focal scale dX or dY "
                "overflow or underflow in fp32");
        }
        if (!isValidBodyToCameraMrp(bodyToCameraMrp)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: bodyToCameraMrp must be finite");
        }
        return {radius,
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
    // Requires the focal scales dX, dY and dX^2, dY^2, dX*dY to be normal fp32 values, so every 1/dX, 1/dY,
    // 1/(dX*dY) and 1/dX^2 in the camera model is finite and nonzero. Defined in cobConverterAlgorithm.cpp.
    static bool isValidFocalScale(float fieldOfViewX, float fieldOfViewY, float resolutionX, float resolutionY);

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
    CobConverterConfig(float radius,
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
        : radius(radius),
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

/*! Phase-angle correction terms, computed every cycle by computePhaseAngleCorrection. */
struct PhaseAngleCorrectionResult {
    Eigen::Vector3d sc_position = Eigen::Vector3d::Zero();
    double spacecraftRange = 0.0;
    Eigen::Vector3f shat_N = Eigen::Vector3f::Zero();
    float alphaPA = 0.0F;
    float phi = 0.0F;
    float gamma = 0.0F;
    float Rc = 0.0F;
    bool validCom = false;  //!< [--] set by updateState, not by computePhaseAngleCorrection
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
    CobConverterUpdateResult updateState(const CobMeasurement& cob,
                                         const VehicleAttitude& attitude,
                                         const FilterState& filter) const;
    int getCameraId() const { return this->cfg.getCameraId(); }
    static UndistortedCoordinate undistortNormalizedCoordinate(float xDistorted,
                                                               float yDistorted,
                                                               const CalibrationCoefficients& coefficients);

   private:
    bool comOutlierDetection(const Eigen::Vector3d& filterVehPosition,
                             const Eigen::Matrix3d& filterVehPositionCovariance,
                             const Eigen::Matrix3f& covar_N,
                             const Eigen::Vector3f& rhatCOM_N) const;
    void computeCameraParameters();
    Rotations computeRotations(const Eigen::Vector3f& sigma_BN) const;
    PhaseAngleCorrectionResult computePhaseAngleCorrection(const Eigen::Vector3d& filterVehPosition,
                                                           const Eigen::Vector3f& vehSunPntBdy,
                                                           const Eigen::Matrix3f& dcm_BN) const;
    float computeBetaVar(const Eigen::Matrix3d& filterVehPositionCovariance,
                         const PhaseAngleCorrectionResult& correction) const;

    CobConverterConfig cfg;
    Eigen::Matrix3f dcm_CB = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrixInverse = Eigen::Matrix3f::Zero();
    float dX{};
    float dY{};
};

#endif  // F32XMERA_COB_CONVERTER_ALGORITHM_H
