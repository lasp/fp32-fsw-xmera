#ifndef F32XMERA_COB_CONVERTER_H
#define F32XMERA_COB_CONVERTER_H

#include <architecture/messaging/messaging.h>
#include <memory>

#include "cobConverterAlgorithm.h"
#include "msgPayloadDef/CameraModelMsgF32Payload.h"
#include "msgPayloadDef/CobConverterDiagnosticMsgF32Payload.h"
#include "msgPayloadDef/FilterMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/OpNavCOBMsgF32Payload.h"
#include "msgPayloadDef/OpNavCOMMsgF32Payload.h"
#include "msgPayloadDef/OpNavUnitVecMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>

/**
 * @enum PhaseAngleCorrectionMethod
 * @brief Phase-angle correction models for converting COB to COM.
 */
enum class PhaseAngleCorrectionMethod { NoCorrection, Binary };

const std::map<PhaseAngleCorrectionMethod, PhaseAngleCorrectionMethodAlgorithm> enumMap = {
    {PhaseAngleCorrectionMethod::NoCorrection, PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg},
    {PhaseAngleCorrectionMethod::Binary, PhaseAngleCorrectionMethodAlgorithm::BinaryAlg}};

/**
 * @class CobConverter
 * @brief Converts center-of-brightness (COB) pixel measurements into unit vectors
 *        (camera, body, inertial frames), with optional phase-angle correction
 *        and outlier detection.
 */
class CobConverter final : public SysModel {
   public:
    CobConverter() = default;
    ~CobConverter() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    // Phase 1: public config properties -- set before reset().
    PhaseAngleCorrectionMethod phaseAngleCorrectionMethod = PhaseAngleCorrectionMethod::NoCorrection;
    float radius = 0.0F;
    float radiusUncertainty = 0.0F;
    Eigen::Matrix3f attitudeCovariance = Eigen::Matrix3f::Zero();
    float numStandardDeviations = 3.0F;
    float standardDeviation = 0.0F;
    bool specifiedStandardDeviation = false;
    bool outlierDetectionEnabled = false;
    CalibrationCoefficients calibrationCoefficients{};
    int cameraId = 0;
    float fieldOfView = 0.0F;

    // Output messages
    Message<OpNavUnitVecMsgF32Payload> opnavUnitVecOutMsg;
    Message<OpNavCOMMsgF32Payload> comCorrectionOutMsg;
    Message<CobConverterDiagnosticMsgF32Payload> cobConverterDiagnosticOutMsg;

    // Input messages
    ReadFunctor<OpNavCOBMsgF32Payload> opnavCOBInMsg;
    ReadFunctor<FilterMsgF32Payload> opnavFilterInMsg;
    ReadFunctor<CameraModelMsgF32Payload> cameraConfigInMsg;
    ReadFunctor<NavAttMsgF32Payload> navAttInMsg;
    ReadFunctor<NavAttMsgF32Payload> sunInMsg;

   private:
    std::unique_ptr<CobConverterAlgorithm> algorithm = nullptr;
};

#endif  // F32XMERA_COB_CONVERTER_H
