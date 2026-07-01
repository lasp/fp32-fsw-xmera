#ifndef F32XMERA_COB_CONVERTER_ALGORITHM_H
#define F32XMERA_COB_CONVERTER_ALGORITHM_H

#include <Eigen/Dense>
#include <cstdint>

/**
 * @brief Camera calibration to pinhole Brown-Conrady coefficients
 */
struct CalibrationCoefficients {
    double k1 = 0;
    double k2 = 0;
    double k3 = 0;
    double p1 = 0;
    double p2 = 0;
};

/**
 * @enum PhaseAngleCorrectionMethodAlgorithm
 * @brief Phase-angle correction models for converting COB to COM.
 */
enum class PhaseAngleCorrectionMethodAlgorithm { NoCorrectionAlg, LambertianAlg, BinaryAlg };

/*! Structure containing all COB converter algorithm inputs. */
struct CobConverterInput {
    uint64_t currentSimNanos{};  //!< [ns]  current simulation time
    // camera model
    Eigen::Vector3d bodyToCameraMrp = Eigen::Vector3d::Zero();  //!< [--] MRP body-to-camera
    double fieldOfView{};                                       //!< [rad] camera field of view
    double resolutionX{};                                       //!< [px]  horizontal resolution
    double resolutionY{};                                       //!< [px]  vertical resolution
    int cameraId{};                                             //!< [--]  camera identifier
    // COB measurement
    bool cobValid{};                                                  //!< [--] validity flag
    int32_t cobPixelsFound{};                                         //!< [--] bright pixels
    Eigen::Vector2d cobCenterOfBrightness = Eigen::Vector2d::Zero();  //!< [px] COB pixel coords
    uint64_t cobTimeTag{};                                            //!< [ns] measurement time
    // navigation attitude
    Eigen::Vector3d sigma_BN = Eigen::Vector3d::Zero();  //!< [--] body-to-inertial MRP
    // sun attitude
    Eigen::Vector3d vehSunPntBdy = Eigen::Vector3d::Zero();  //!< [--] sun direction, body frame
    // filter state
    Eigen::VectorXd filterState;       //!< [--] filter state vector
    Eigen::MatrixXd filterCovariance;  //!< [--] filter covariance matrix
};

/*! Structure containing all COB converter algorithm outputs. */
struct CobConverterOutput {
    Eigen::Matrix3d covar_N = Eigen::Matrix3d::Zero();             //!< [--] COM covariance, inertial frame
    Eigen::Matrix3d covar_C = Eigen::Matrix3d::Zero();             //!< [--] COM covariance, camera frame
    Eigen::Matrix3d covar_B = Eigen::Matrix3d::Zero();             //!< [--] COM covariance, body frame
    Eigen::Vector3d rhat_BN_N = Eigen::Vector3d::Zero();           //!< [--] COM unit vector, inertial frame
    Eigen::Vector3d rhat_BN_C = Eigen::Vector3d::Zero();           //!< [--] COM unit vector, camera frame
    Eigen::Vector3d rhat_BN_B = Eigen::Vector3d::Zero();           //!< [--] COM unit vector, body frame
    double unitVecTimeTag{};                                       //!< [s]  measurement timestamp
    bool unitVecValid{};                                           //!< [--] COM unit vector validity flag
    Eigen::Vector2d centerOfBrightness = Eigen::Vector2d::Zero();  //!< [px] COB pixel coordinates
    Eigen::Vector2d centerOfMass = Eigen::Vector2d::Zero();        //!< [px] COM pixel coordinates
    double offsetFactor{};                                         //!< [--] phase-angle offset factor (gamma)
    int objectPixelRadius{};                                       //!< [px] object radius in pixels
    double phaseAngle{};                                           //!< [rad] phase angle alpha_PA
    double sunDirection{};                                         //!< [rad] sun direction phi in image plane
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
    CobConverterAlgorithm(PhaseAngleCorrectionMethodAlgorithm method, double radiusObject);
    ~CobConverterAlgorithm();

    CobConverterOutput updateState(const CobConverterInput& input);

    void setRadius(double radius);
    double getRadius() const;
    void setRadiusUncertainty(double radiusUncertainty);
    double getRadiusUncertainty() const;
    void setAttitudeCovariance(const Eigen::Matrix3d& covAtt_BN_B);
    Eigen::Matrix3d getAttitudeCovariance() const;
    void setNumStandardDeviations(double num);
    double getNumStandardDeviations() const;
    void setStandardDeviation(double num);
    double getStandardDeviation() const;
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
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> computeCentersOfInterest(const CobConverterInput& input) const;
    void computeRelevantVectors(const Eigen::Vector3d& centerOfBrightness, const Eigen::Vector3d& centerOfMass);
    void computeCameraFrameUncertainty(const CobConverterInput& input);
    Eigen::Vector3d calibrateDistortions(const Eigen::Vector3d& unCalibratedVector) const;
    void populateOutputMessages(uint64_t timeTag,
                                const Eigen::Vector3d& centerOfMass,
                                const Eigen::Vector3d& centerOfBrightness,
                                CobConverterOutput& output) const;

    PhaseAngleCorrectionMethodAlgorithm phaseAngleCorrectionMethod =
        PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg;
    CalibrationCoefficients calibrationCoefficients{};
    double objectRadius{};
    double objectRadiusUncertainty{};
    Eigen::Matrix3d covarAtt_BN_B = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dcm_NC = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dcm_CB = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d dcm_BN = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d cameraCalibrationMatrix = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d cameraCalibrationMatrixInverse = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d covar_B = Eigen::Matrix3d::Zero();
    double numStandardDeviations = 3;
    double standardDeviation{};
    bool specifiedStandardDeviation{};
    bool performOutlierDetection{};
    bool validCOM = false;
    double dX{};
    double X{};
    double Y{};
    double ifov_x{};
    double ifov_y{};
    double Rc = 0;
    double gamma = 0;
    double phi = 0;
    double alphaPA = 0;
    Eigen::Vector3d rhatCOB_C = Eigen::Vector3d::Zero();
    Eigen::Vector3d rhatCOM_C = Eigen::Vector3d::Zero();
    Eigen::Vector3d sc_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d shat_N = Eigen::Vector3d::Zero();
    double rhatCOBNorm = 0;
    double spacecraftRange = 0;
    int cameraId = 0;
    bool goodOutlierCheck = true;
};

#endif  // F32XMERA_COB_CONVERTER_ALGORITHM_H
