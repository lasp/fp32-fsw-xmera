#ifndef F32XMERA_COB_CONVERTER_H
#define F32XMERA_COB_CONVERTER_H

#include <architecture/messaging/messaging.h>
#include <memory>

#include "cobConverterAlgorithm.h"
#include "msgPayloadDef/CobConverterDiagnosticMsgF32Payload.h"
#include "msgPayloadDef/FilterMsgF32Payload.h"
#include "msgPayloadDef/NavAttMsgF32Payload.h"
#include "msgPayloadDef/OpNavCOBMsgF32Payload.h"
#include "msgPayloadDef/OpNavUnitVecMsgF32Payload.h"
#include <architecture/_GeneralModuleFiles/sys_model.h>

/**
 * @class CobConverter
 * @brief Converts center-of-brightness (COB) pixel measurements into unit vectors
 *        (camera, body, inertial frames), applying the Binary phase-angle correction
 *        and optional outlier detection.
 */
class CobConverter final : public SysModel {
   public:
    CobConverter() = default;
    ~CobConverter() override = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void reconfigure() const;

    // Phase 1: public config properties -- set before reset().
    float radius = 0.0F;
    float radiusUncertainty = 0.0F;
    Eigen::Matrix3f attitudeCovariance = Eigen::Matrix3f::Zero();
    float numStandardDeviations = 3.0F;
    float standardDeviation = 0.0F;
    bool specifiedStandardDeviation = false;
    bool outlierDetectionEnabled = false;
    CalibrationCoefficients calibrationCoefficients{};
    int cameraId = 0;
    float fieldOfViewX = 0.0F;
    float fieldOfViewY = 0.0F;
    float resolutionX = 0.0F;
    float resolutionY = 0.0F;
    Eigen::Vector3f bodyToCameraMrp = Eigen::Vector3f::Zero();

    // Output messages
    Message<OpNavUnitVecMsgF32Payload> opnavUnitVecOutMsg;
    Message<CobConverterDiagnosticMsgF32Payload> cobConverterDiagnosticOutMsg;

    // Input messages
    ReadFunctor<OpNavCOBMsgF32Payload> opnavCOBInMsg;
    ReadFunctor<FilterMsgF32Payload> opnavFilterInMsg;
    ReadFunctor<NavAttMsgF32Payload> navAttInMsg;  //!< attitude and sun direction (e.g. navAggregate.navAttOutMsg)

   private:
    CobConverterConfig toConfig() const;
    std::unique_ptr<CobConverterAlgorithm> algorithm = nullptr;
};

#endif  // F32XMERA_COB_CONVERTER_H
