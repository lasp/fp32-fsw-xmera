#ifndef F32XMERA_COB_CONVERTER_ALGORITHM_H
#define F32XMERA_COB_CONVERTER_ALGORITHM_H

#include <Eigen/Dense>
#include <cstdint>

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
enum class PhaseAngleCorrectionMethodAlgorithm { NoCorrectionAlg, BinaryAlg };

/*! Structure containing all COB converter algorithm inputs. */
struct CobConverterInput {
    // camera model
    Eigen::Vector3f bodyToCameraMrp = Eigen::Vector3f::Zero();  //!< [--] MRP body-to-camera
    float fieldOfView{};                                        //!< [rad] camera field of view
    float resolutionX{};                                        //!< [px]  horizontal resolution
    float resolutionY{};                                        //!< [px]  vertical resolution
    int cameraId{};                                             //!< [--]  camera identifier
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
    int cameraID{};                                                //!< [--] camera identifier
    uint64_t comTimeTag{};                                         //!< [ns] measurement timestamp
    bool comValid{};                                               //!< [--] COM validity flag
    bool coberrorOutlierTrigger{};  //!< [--] true if COB error exceeded outlier threshold
};

/**
 * @class CobConverterAlgorithm
 * @brief Converts center-of-brightness (COB) pixel measurements into unit vectors
 *        (camera, body, inertial frames), with optional phase-angle correction
 *        and outlier detection.
 */
class CobConverterAlgorithm {
   public:
    CobConverterAlgorithm(PhaseAngleCorrectionMethodAlgorithm method, float radiusObject);
    ~CobConverterAlgorithm();

    CobConverterOutput updateState(const CobConverterInput& input);

    void setRadius(float radius);
    float getRadius() const;
    void setRadiusUncertainty(float radiusUncertainty);
    float getRadiusUncertainty() const;
    void setAttitudeCovariance(const Eigen::Matrix3f& covAtt_BN_B);
    Eigen::Matrix3f getAttitudeCovariance() const;
    void setNumStandardDeviations(float num);
    float getNumStandardDeviations() const;
    void setStandardDeviation(float num);
    float getStandardDeviation() const;
    bool isStandardDeviationSpecified() const;
    void enableOutlierDetection();
    void disableOutlierDetection();
    bool isOutlierDetectionEnabled() const;
    void setBrownConradyCoefficients(const CalibrationCoefficients& coefficients);
    CalibrationCoefficients getBrownConradyCoefficients() const;

   private:
    void cobOutlierDetection(const CobConverterInput& input, CobConverterOutput& output);
    void computeCameraParameters(const CobConverterInput& input);
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

    PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod =
        PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg;
    CalibrationCoefficients calibrationCoefficients{};
    float objectRadius{};
    float objectRadiusUncertainty{};
    Eigen::Matrix3f covarAtt_BN_B = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f dcm_NC = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f dcm_CB = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f dcm_BN = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f cameraCalibrationMatrixInverse = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f covar_B = Eigen::Matrix3f::Zero();
    float numStandardDeviations = 3;
    float standardDeviation{};
    bool specifiedStandardDeviation{};
    bool performOutlierDetection{};
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
    int cameraId = 0;
    bool goodOutlierCheck = true;
};

#endif  // F32XMERA_COB_CONVERTER_ALGORITHM_H
