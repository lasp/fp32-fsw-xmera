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
    if (!this->navAttInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.navAttInMsg wasn't connected.");
    }
    if (!this->sunInMsg.isLinked()) {
        throw std::invalid_argument("CobConverter.sunInMsg wasn't connected.");
    }
    if (this->opnavFilterInMsg.isLinked() && this->opnavFilterInMsg().numberOfStates != 6) {
        throw std::invalid_argument("CobConverter.opnavFilterInMsg: numberOfStates must be 6.");
    }

    this->algorithm = std::make_unique<CobConverterAlgorithm>(this->toConfig());
}

/**
 * @brief Build a validated CobConverterConfig from the adapter's stored properties.
 * @return CobConverterConfig validated configuration.
 */
CobConverterConfig CobConverter::toConfig() const {
    return CobConverterConfig::create(enumMap.at(this->phaseAngleCorrectionMethod),
                                      this->radius,
                                      this->radiusUncertainty,
                                      this->attitudeCovariance,
                                      this->numStandardDeviations,
                                      this->standardDeviation,
                                      this->specifiedStandardDeviation,
                                      this->outlierDetectionEnabled,
                                      this->calibrationCoefficients,
                                      this->cameraId,
                                      this->fieldOfViewX,
                                      this->fieldOfViewY,
                                      this->resolutionX,
                                      this->resolutionY,
                                      this->bodyToCameraMrp);
}

/**
 * @brief Push a fresh configuration into the algorithm without reconstructing it.
 */
void CobConverter::reconfigure() const {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CobConverter reset() has not been called.");
    }
    this->algorithm->setConfig(this->toConfig());
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

    const OpNavCOBMsgF32Payload cobMsg = this->opnavCOBInMsg();
    const NavAttMsgF32Payload navAttMsg = this->navAttInMsg();
    const NavAttMsgF32Payload sunMsg = this->sunInMsg();
    const FilterMsgF32Payload filterMsg = this->opnavFilterInMsg();

    CobMeasurement cob;
    cob.cobValid = cobMsg.valid;
    cob.cobPixelsFound = cobMsg.pixelsFound;
    cob.cobCenterOfBrightness = Eigen::Map<const Eigen::Vector2f>(cobMsg.centerOfBrightness);
    cob.cobTimeTag = cobMsg.timeTag;

    VehicleAttitude attitude;
    attitude.sigma_BN = cArrayToEigenVector(navAttMsg.sigma_BN);
    attitude.vehSunPntBdy = cArrayToEigenVector(sunMsg.vehSunPntBdy);

    FilterState filter;
    filter.filterVehPosition = cArrayToEigenVector3<double>(filterMsg.state);
    filter.filterVehPositionCovariance = cArrayToEigenMatrix<double, 6, 6>(filterMsg.covar).topLeftCorner<3, 3>();

    const CobConverterOutput out = this->algorithm->updateState(cob, attitude, filter);

    OpNavUnitVecMsgF32Payload uVecOutMsgBuffer{};
    eigenMatrixToCArray(out.unitVec.covar_N, uVecOutMsgBuffer.covar_N);
    eigenMatrixToCArray(out.unitVec.covar_C, uVecOutMsgBuffer.covar_C);
    eigenMatrixToCArray(out.unitVec.covar_B, uVecOutMsgBuffer.covar_B);
    eigenVectorToCArray(out.unitVec.rhat_BN_N, uVecOutMsgBuffer.rhat_BN_N);
    eigenVectorToCArray(out.unitVec.rhat_BN_C, uVecOutMsgBuffer.rhat_BN_C);
    eigenVectorToCArray(out.unitVec.rhat_BN_B, uVecOutMsgBuffer.rhat_BN_B);
    uVecOutMsgBuffer.timeTag = out.unitVec.unitVecTimeTag;
    uVecOutMsgBuffer.valid = out.unitVec.unitVecValid;

    OpNavCOMMsgF32Payload comMsgBuffer{};
    comMsgBuffer.centerOfBrightness[0] = out.com.centerOfBrightness[0];
    comMsgBuffer.centerOfBrightness[1] = out.com.centerOfBrightness[1];
    comMsgBuffer.centerOfMass[0] = out.com.centerOfMass[0];
    comMsgBuffer.centerOfMass[1] = out.com.centerOfMass[1];
    comMsgBuffer.offsetFactor = out.com.offsetFactor;
    comMsgBuffer.objectPixelRadius = out.com.objectPixelRadius;
    comMsgBuffer.phaseAngle = out.com.phaseAngle;
    comMsgBuffer.sunDirection = out.com.sunDirection;
    comMsgBuffer.cameraID = this->algorithm->getCameraId();
    comMsgBuffer.timeTag = out.com.comTimeTag;
    comMsgBuffer.valid = out.com.comValid;

    CobConverterDiagnosticMsgF32Payload diagnosticMsgBuffer{};
    diagnosticMsgBuffer.coberrorOutlierTrigger = out.diagnostic.coberrorOutlierTrigger;

    this->opnavUnitVecOutMsg.write(uVecOutMsgBuffer, this->moduleID, currentSimNanos);
    this->comCorrectionOutMsg.write(comMsgBuffer, this->moduleID, currentSimNanos);
    this->cobConverterDiagnosticOutMsg.write(diagnosticMsgBuffer, this->moduleID, currentSimNanos);
}
