#include "cobConverter.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/xmera/xmeraLifecycleException.h"
#include <stdexcept>

/**
 * @brief Reset internal state, validate required input connections, and construct the algorithm from the
 *        currently-set config properties.
 * @param currentSimNanos Current simulation time in nanoseconds.
 * @throws std::invalid_argument If any required input message link is missing, or if a config property
 *         is invalid.
 */
void CobConverter::reset(uint64_t currentSimNanos) {
    // throw if any required message is not connected
    if (!this->opnavCOBInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.opnavCOBInMsg wasn't connected.");
    }
    if (this->outlierDetectionEnabled && !this->opnavFilterInMsg.isLinked()) {
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
    if (this->opnavFilterInMsg.isLinked() && this->opnavFilterInMsg().numberOfStates != 6) {
        throw std::invalid_argument("CobConverter.opnavFilterInMsg: numberOfStates must be 6.");
    }

    const auto config = CobConverterConfig::create(enumMap.at(this->phaseAngleCorrectionMethod),
                                                   this->radius,
                                                   this->radiusUncertainty,
                                                   this->attitudeCovariance,
                                                   this->numStandardDeviations,
                                                   this->standardDeviation,
                                                   this->specifiedStandardDeviation,
                                                   this->outlierDetectionEnabled,
                                                   this->calibrationCoefficients);
    this->algorithm = std::make_unique<CobConverterAlgorithm>(config);
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
    if (!this->algorithm) {
        throw XmeraLifecycleException("CobConverter reset() has not been called.");
    }

    const CameraModelMsgF32Payload cameraMsg = this->cameraConfigInMsg();
    const OpNavCOBMsgF32Payload cobMsg = this->opnavCOBInMsg();
    const NavAttMsgF32Payload navAttMsg = this->navAttInMsg();
    const NavAttMsgF32Payload sunMsg = this->sunInMsg();
    const FilterMsgF32Payload filterMsg = this->opnavFilterInMsg();

    CobConverterInput input;
    input.bodyToCameraMrp = cArrayToEigenVector(cameraMsg.bodyToCameraMrp);
    input.fieldOfView = cameraMsg.fieldOfView[0];
    input.resolutionX = cameraMsg.resolution[0];
    input.resolutionY = cameraMsg.resolution[1];
    input.cameraId = cameraMsg.cameraId;
    input.cobValid = cobMsg.valid;
    input.cobPixelsFound = cobMsg.pixelsFound;
    input.cobCenterOfBrightness = Eigen::Map<const Eigen::Vector2f>(cobMsg.centerOfBrightness);
    input.cobTimeTag = cobMsg.timeTag;
    input.sigma_BN = cArrayToEigenVector(navAttMsg.sigma_BN);
    input.vehSunPntBdy = cArrayToEigenVector(sunMsg.vehSunPntBdy);
    input.filterVehPosition = cArrayToEigenVector3<double>(filterMsg.state);
    input.filterVehPositionCovariance = cArrayToEigenMatrix<double, 6, 6>(filterMsg.covar).topLeftCorner<3, 3>();

    const CobConverterOutput out = this->algorithm->updateState(input);

    OpNavUnitVecMsgF32Payload uVecOutMsgBuffer{};
    eigenMatrixToCArray(out.covar_N, uVecOutMsgBuffer.covar_N);
    eigenMatrixToCArray(out.covar_C, uVecOutMsgBuffer.covar_C);
    eigenMatrixToCArray(out.covar_B, uVecOutMsgBuffer.covar_B);
    eigenVectorToCArray(out.rhat_BN_N, uVecOutMsgBuffer.rhat_BN_N);
    eigenVectorToCArray(out.rhat_BN_C, uVecOutMsgBuffer.rhat_BN_C);
    eigenVectorToCArray(out.rhat_BN_B, uVecOutMsgBuffer.rhat_BN_B);
    uVecOutMsgBuffer.timeTag = out.unitVecTimeTag;
    uVecOutMsgBuffer.valid = out.unitVecValid;

    OpNavCOMMsgF32Payload comMsgBuffer{};
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

    CobConverterDiagnosticMsgF32Payload diagnosticMsgBuffer{};
    diagnosticMsgBuffer.coberrorOutlierTrigger = out.coberrorOutlierTrigger;

    this->opnavUnitVecOutMsg.write(&uVecOutMsgBuffer, this->moduleID, currentSimNanos);
    this->comCorrectionOutMsg.write(&comMsgBuffer, this->moduleID, currentSimNanos);
    this->cobConverterDiagnosticOutMsg.write(&diagnosticMsgBuffer, this->moduleID, currentSimNanos);
}
