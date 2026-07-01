#include "cobConverter.h"
#include <stdexcept>
#include "utilities/fsw/eigenSupport.h"

/**
 * @brief Construct a CobConverter.
 * @param method Phase-angle correction method to apply.
 * @param radiusObject Object radius in meters (must be > 0).
 * @throws std::invalid_argument If radiusObject is not > 0.
 */
CobConverter::CobConverter(const PhaseAngleCorrectionMethod method, const double radiusObject)
    : algorithm(enumMap.at(method), radiusObject) {}

/** @brief Default destructor. */
CobConverter::~CobConverter() = default;

/**
 * @brief Reset internal state and validate required input connections.
 * @param currentSimNanos Current simulation time in nanoseconds.
 * @throws std::invalid_argument If any required input message link is missing.
 */
void CobConverter::reset(uint64_t currentSimNanos) {
    // throw if any required message is not connected
    if (!this->opnavCOBInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.opnavCOBInMsg wasn't connected.");
    }
    if (this->algorithm.isOutlierDetectionEnabled() && !this->opnavFilterInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.opnavFilterInMsg wasn't connected.");
    }
    if (!this->cameraConfigInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.cameraConfigInMsg wasn't connected.");
    }
    if (!this->navAttInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.navAttInMsg wasn't connected.");
    }
    if (!this->sunInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.sunInMsg wasn't connected.");
    }
}

/**
 * @brief Update step: convert pixel-based COB into unit vectors and outputs.
 *
 * Reads inputs, computes parameters and corrections, performs optional outlier
 * detection, and writes out three payloads: COB unit vector, COM unit vector, and
 * COM metadata.
 *
 * @param currentSimNanos Current simulation time in nanoseconds.
 */
void CobConverter::updateState(const uint64_t currentSimNanos) {
    const CameraModelMsgPayload cameraMsg = this->cameraConfigInMsg();
    const OpNavCOBMsgPayload cobMsg = this->opnavCOBInMsg();
    const NavAttMsgPayload navAttMsg = this->navAttInMsg();
    const NavAttMsgPayload sunMsg = this->sunInMsg();
    const FilterMsgPayload filterMsg = this->opnavFilterInMsg();

    CobConverterInput input;
    input.currentSimNanos = currentSimNanos;
    input.bodyToCameraMrp = Eigen::Map<const Eigen::Vector3d>(cameraMsg.bodyToCameraMrp);
    input.fieldOfView = cameraMsg.fieldOfView[0];
    input.resolutionX = cameraMsg.resolution[0];
    input.resolutionY = cameraMsg.resolution[1];
    input.cameraId = cameraMsg.cameraId;
    input.cobValid = cobMsg.valid;
    input.cobPixelsFound = cobMsg.pixelsFound;
    input.cobCenterOfBrightness = Eigen::Map<const Eigen::Vector2d>(cobMsg.centerOfBrightness);
    input.cobTimeTag = cobMsg.timeTag;
    input.sigma_BN = Eigen::Map<const Eigen::Vector3d>(navAttMsg.sigma_BN);
    input.vehSunPntBdy = Eigen::Map<const Eigen::Vector3d>(sunMsg.vehSunPntBdy);
    input.filterState = Eigen::Map<const Eigen::VectorXd>(filterMsg.state, filterMsg.numberOfStates);
    input.filterCovariance =
        Eigen::Map<const Eigen::MatrixXd>(filterMsg.covar, filterMsg.numberOfStates, filterMsg.numberOfStates);

    const CobConverterOutput out = this->algorithm.updateState(input);

    OpNavUnitVecMsgPayload uVecOutMsgBuffer{};
    eigenMatrixToCArray(out.covar_N, uVecOutMsgBuffer.covar_N);
    eigenMatrixToCArray(out.covar_C, uVecOutMsgBuffer.covar_C);
    eigenMatrixToCArray(out.covar_B, uVecOutMsgBuffer.covar_B);
    eigenVectorToCArray(out.rhat_BN_N, uVecOutMsgBuffer.rhat_BN_N);
    eigenVectorToCArray(out.rhat_BN_C, uVecOutMsgBuffer.rhat_BN_C);
    eigenVectorToCArray(out.rhat_BN_B, uVecOutMsgBuffer.rhat_BN_B);
    uVecOutMsgBuffer.timeTag = out.unitVecTimeTag;
    uVecOutMsgBuffer.valid = out.unitVecValid;

    OpNavCOMMsgPayload comMsgBuffer{};
    comMsgBuffer.centerOfBrightness[0] = out.centerOfBrightness[0];
    comMsgBuffer.centerOfBrightness[1] = out.centerOfBrightness[1];
    comMsgBuffer.centerOfMass[0] = out.centerOfMass[0];
    comMsgBuffer.centerOfMass[1] = out.centerOfMass[1];
    comMsgBuffer.offsetFactor = out.offsetFactor;
    comMsgBuffer.objectPixelRadius = out.objectPixelRadius;
    comMsgBuffer.phaseAngle = out.phaseAngle;
    comMsgBuffer.sunDirection = out.sunDirection;
    comMsgBuffer.cameraID = out.cameraID;
    comMsgBuffer.timeTag = out.comTimeTag;
    comMsgBuffer.valid = out.comValid;

    CobConverterDiagnosticMsgPayload diagnosticMsgBuffer{};
    diagnosticMsgBuffer.coberrorOutlierTrigger = out.coberrorOutlierTrigger;

    this->opnavUnitVecOutMsg.write(&uVecOutMsgBuffer, this->moduleID, currentSimNanos);
    this->comCorrectionOutMsg.write(&comMsgBuffer, this->moduleID, currentSimNanos);
    this->cobConverterDiagnosticOutMsg.write(&diagnosticMsgBuffer, this->moduleID, currentSimNanos);
}

/**
 * @brief Set the object radius.
 * @param radius Object radius in meters (must be > 0).
 * @throws std::invalid_argument If radius is not > 0.
 */
void CobConverter::setRadius(const double radius) {
    if (radius <= 0) {
        throw std::invalid_argument("cobConverter: radius must be > 0");
    }
    this->algorithm.setRadius(radius);
}

/**
 * @brief Get the object radius.
 * @return Object radius in meters.
 */
double CobConverter::getRadius() const {
    return this->algorithm.getRadius();
}

/**
 * @brief Set the object radius uncertainty.
 * @param radiusUncertainty Object radius uncertainty in meters (>= 0).
 * @throws std::invalid_argument If radiusUncertainty is < 0.
 */
void CobConverter::setRadiusUncertainty(const double radiusUncertainty) {
    if (radiusUncertainty < 0) {
        throw std::invalid_argument("cobConverter: radiusUncertainty must be >= 0");
    }
    this->algorithm.setRadiusUncertainty(radiusUncertainty);
}

/**
 * @brief Get the object radius uncertainty.
 * @return Object radius uncertainty in meters.
 */
double CobConverter::getRadiusUncertainty() const { return this->algorithm.getRadiusUncertainty(); }

/**
 * @brief Set the attitude error covariance matrix in body frame (for unit vector measurements).
 * @param covAtt_BN_B 3x3 attitude covariance in body frame.
 */
void CobConverter::setAttitudeCovariance(const Eigen::Matrix3d& covAtt_BN_B) {
    this->algorithm.setAttitudeCovariance(covAtt_BN_B);
}

/**
 * @brief Get the attitude error covariance matrix in body frame (for unit vector measurements).
 * @return 3x3 attitude covariance in body frame.
 */
Eigen::Matrix3d CobConverter::getAttitudeCovariance() const { return this->algorithm.getAttitudeCovariance(); }

/**
 * @brief Set the number of standard deviations for outlier gating.
 * @param num Number of sigmas (> 0).
 * @throws std::invalid_argument If num is not > 0.
 */
void CobConverter::setNumStandardDeviations(const double num) {
    if (num <= 0.0) {
        throw std::invalid_argument("cobConverter: numStandardDeviations must be > 0");
    }
    this->algorithm.setNumStandardDeviations(num);
}

/**
 * @brief Get the configured number of standard deviations for outlier gating.
 * @return Number of sigmas.
 */
double CobConverter::getNumStandardDeviations() const { return this->algorithm.getNumStandardDeviations(); }

/**
 * @brief Set an explicit standard deviation for the expected COB error.
 * @param num Standard deviation (> 0).
 * @note When set, outlier detection will use this fixed value instead of deriving one.
 * @throws std::invalid_argument If num is not > 0.
 */
void CobConverter::setStandardDeviation(const double num) {
    if (num <= 0.0) {
        throw std::invalid_argument("cobConverter: standardDeviation must be > 0");
    }
    this->algorithm.setStandardDeviation(num);
}

/**
 * @brief Get the explicitly specified standard deviation (if set).
 * @return Standard deviation value.
 */
double CobConverter::getStandardDeviation() const { return this->algorithm.getStandardDeviation(); }

/**
 * @brief Determine whether a standard deviation has been explicitly specified.
 * @return True if specified, false otherwise.
 */
bool CobConverter::isStandardDeviationSpecified() const { return this->algorithm.isStandardDeviationSpecified(); }

/**
 * @brief Enable COB outlier detection.
 */
void CobConverter::enableOutlierDetection() { this->algorithm.enableOutlierDetection(); }

/**
 * @brief Disable COB outlier detection.
 */
void CobConverter::disableOutlierDetection() { this->algorithm.disableOutlierDetection(); }

/**
 * @brief Check whether COB outlier detection is enabled.
 * @return True if enabled, false otherwise.
 */
bool CobConverter::isOutlierDetectionEnabled() const { return this->algorithm.isOutlierDetectionEnabled(); }

/**
 * @brief Set the Brown-Conrady coefficients.
 * @param coefficients CalibrationCoefficients
 */
void CobConverter::setBrownConradyCoefficients(const CalibrationCoefficients& coefficients) {
    this->algorithm.setBrownConradyCoefficients(coefficients);
}

/**
 * @brief Get the Brown-Conrady coefficients.
 * @return CalibrationCoefficients
 */
CalibrationCoefficients CobConverter::getBrownConradyCoefficients() const {
    return this->algorithm.getBrownConradyCoefficients();
}
