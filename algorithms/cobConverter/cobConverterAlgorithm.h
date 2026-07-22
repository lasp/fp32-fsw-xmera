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

/*! Structure containing all COB converter algorithm outputs. */
struct CobConverterOutput {
    Eigen::Matrix3f covar_N = Eigen::Matrix3f::Zero();             //!< [--] COM covariance, inertial frame
    Eigen::Matrix3f covar_C = Eigen::Matrix3f::Zero();             //!< [--] COM covariance, camera frame
    Eigen::Matrix3f covar_B = Eigen::Matrix3f::Zero();             //!< [--] COM covariance, body frame
    Eigen::Vector3f rhat_BN_N = Eigen::Vector3f::Zero();           //!< [--] COM unit vector, inertial frame
    Eigen::Vector3f rhat_BN_C = Eigen::Vector3f::Zero();           //!< [--] COM unit vector, camera frame
    Eigen::Vector3f rhat_BN_B = Eigen::Vector3f::Zero();           //!< [--] COM unit vector, body frame
    double unitVecTimeTag{};                                       //!< [s]  measurement timestamp
    bool unitVecValid{};                                           //!< [--] COM unit vector validity flag
    Eigen::Vector2f centerOfBrightness = Eigen::Vector2f::Zero();  //!< [px] COB pixel coordinates
    Eigen::Vector2f centerOfMass = Eigen::Vector2f::Zero();        //!< [px] COM pixel coordinates
    float offsetFactor{};                                          //!< [--] phase-angle offset factor (gamma)
    int objectPixelRadius{};                                       //!< [px] object radius in pixels
    float phaseAngle{};                                            //!< [rad] phase angle alpha_PA
    float sunDirection{};                                          //!< [rad] sun direction phi in image plane
    uint64_t comTimeTag{};                                         //!< [ns] measurement timestamp
    bool comValid{};                                               //!< [--] COM validity flag
    bool coberrorOutlierTrigger{};  //!< [--] true if COB error exceeded outlier threshold
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
                                     float fieldOfView,
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
        if (!isValidFieldOfView(fieldOfView)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: fieldOfView must be > 0 and < pi");
        }
        if (!isValidResolutionX(resolutionX)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: resolutionX must be > 0");
        }
        if (!isValidResolutionY(resolutionY)) {
            FSW_THROW_INVALID_ARGUMENT("cobConverter: resolutionY must be > 0");
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
                fieldOfView,
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
    float getFieldOfView() const { return fieldOfView; }
    float getResolutionX() const { return resolutionX; }
    float getResolutionY() const { return resolutionY; }
    Eigen::Vector3f getBodyToCameraMrp() const { return bodyToCameraMrp; }

   private:
    CobConverterConfig(PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod,
                       float radius,  // NOLINT(bugprone-easily-swappable-parameters)
                       float radiusUncertainty,
                       const Eigen::Matrix3f& attitudeCovariance,  // NOLINT(modernize-pass-by-value)
                       float numStandardDeviations,                // NOLINT(bugprone-easily-swappable-parameters)
                       float standardDeviation,
                       bool specifiedStandardDeviation,
                       bool outlierDetectionEnabled,
                       const CalibrationCoefficients& calibrationCoefficients,
                       int cameraId,
                       float fieldOfView,
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
          fieldOfView(fieldOfView),
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
    float fieldOfView;
    float resolutionX;
    float resolutionY;
    Eigen::Vector3f bodyToCameraMrp;
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
    CobConverterOutput updateState(const CobConverterInput& input);
    int getCameraId() const { return this->cfg.getCameraId(); }

   private:
    void cobOutlierDetection(const CobConverterInput& input, CobConverterOutput& output);
    void computeCameraParameters();
    void computeRotations(const CobConverterInput& input);
    void computePhaseAngleCorrection(const CobConverterInput& input);
    std::tuple<Eigen::Vector3f, Eigen::Vector3f> computeCentersOfInterest(const CobConverterInput& input) const;
    void computeRelevantVectors(const Eigen::Vector3f& centerOfBrightness, const Eigen::Vector3f& centerOfMass);
    void computeCameraFrameUncertainty(const CobConverterInput& input);
    Eigen::Vector3f calibrateDistortions(const Eigen::Vector3f& unCalibratedVector) const;
    void populateOutputMessages(uint64_t timeTag,
                                const Eigen::Vector3f& centerOfMass,
                                const Eigen::Vector3f& centerOfBrightness,
                                CobConverterOutput& output) const;

    CobConverterConfig cfg;
    Eigen::Matrix3f dcm_NC = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f dcm_CB = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f dcm_BN = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrixInverse = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f covar_B = Eigen::Matrix3f::Zero();
    bool validCOM = false;
    float dX{};
    float X{};
    float Y{};
    float ifov_x{};
    float ifov_y{};
    float Rc = 0.0F;
    float gamma = 0.0F;
    float phi = 0.0F;
    float alphaPA = 0.0F;
    Eigen::Vector3f rhatCOB_C = Eigen::Vector3f::Zero();
    Eigen::Vector3f rhatCOM_C = Eigen::Vector3f::Zero();
    Eigen::Vector3d sc_position = Eigen::Vector3d::Zero();
    Eigen::Vector3f shat_N = Eigen::Vector3f::Zero();
    double spacecraftRange = 0;
    bool goodOutlierCheck = true;
};

#endif  // F32XMERA_COB_CONVERTER_ALGORITHM_H
