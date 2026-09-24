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
    return CobConverterConfig::create(this->radius,
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
    const FilterMsgF32Payload filterMsg = this->opnavFilterInMsg();

    CobMeasurement cob;
    cob.cobValid = cobMsg.valid;
    cob.cobPixelsFound = cobMsg.pixelsFound;
    cob.cobCenterOfBrightness = Eigen::Map<const Eigen::Vector2f>(cobMsg.centerOfBrightness);
    cob.cobTimeTag = cobMsg.timeTag;

    VehicleAttitude attitude;
    attitude.sigma_BN = cArrayToEigenVector(navAttMsg.sigma_BN);
    attitude.vehSunPntBdy = cArrayToEigenVector(navAttMsg.vehSunPntBdy);

    FilterState filter;
    filter.filterVehPosition = cArrayToEigenVector3<double>(filterMsg.state);
    filter.filterVehPositionCovariance = cArrayToEigenMatrix<double, 6, 6>(filterMsg.covar).topLeftCorner<3, 3>();

    const auto [out, diag] = this->algorithm->updateState(cob, attitude, filter);

    OpNavUnitVecMsgF32Payload uVecOutMsgBuffer{};
    eigenMatrixToCArray(out.covar_N, uVecOutMsgBuffer.covar_N);
    eigenVectorToCArray(out.rhat_BN_N, uVecOutMsgBuffer.rhat_BN_N);
    uVecOutMsgBuffer.timeTag = out.unitVecTimeTag;
    uVecOutMsgBuffer.valid = out.unitVecValid;

    CobConverterDiagnosticMsgF32Payload diagnosticMsgBuffer{};
    eigenMatrixToCArray(diag.covar_C, diagnosticMsgBuffer.covar_C);
    eigenMatrixToCArray(diag.covar_B, diagnosticMsgBuffer.covar_B);
    eigenVectorToCArray(diag.rhat_BN_C, diagnosticMsgBuffer.rhat_BN_C);
    eigenVectorToCArray(diag.rhat_BN_B, diagnosticMsgBuffer.rhat_BN_B);
    eigenVectorToCArray(diag.rhat_COB_C, diagnosticMsgBuffer.rhat_COB_C);
    eigenVectorToCArray(diag.rhat_COB_N, diagnosticMsgBuffer.rhat_COB_N);
    diagnosticMsgBuffer.centerOfBrightness[0] = diag.centerOfBrightness[0];
    diagnosticMsgBuffer.centerOfBrightness[1] = diag.centerOfBrightness[1];
    diagnosticMsgBuffer.centerOfMass[0] = diag.centerOfMass[0];
    diagnosticMsgBuffer.centerOfMass[1] = diag.centerOfMass[1];
    diagnosticMsgBuffer.offsetFactor = diag.offsetFactor;
    diagnosticMsgBuffer.objectPixelRadius = diag.objectPixelRadius;
    diagnosticMsgBuffer.phaseAngle = diag.phaseAngle;
    diagnosticMsgBuffer.sunDirection = diag.sunDirection;
    diagnosticMsgBuffer.cameraID = this->algorithm->getCameraId();
    diagnosticMsgBuffer.comTimeTag = diag.comTimeTag;
    diagnosticMsgBuffer.comValid = diag.comValid;
    diagnosticMsgBuffer.comErrorOutlierTrigger = diag.comErrorOutlierTrigger;

    this->opnavUnitVecOutMsg.write(uVecOutMsgBuffer, this->moduleID, currentSimNanos);
    this->cobConverterDiagnosticOutMsg.write(diagnosticMsgBuffer, this->moduleID, currentSimNanos);
}
