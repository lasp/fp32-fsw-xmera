#ifndef F32XMERA_COB_CONVERTER_H
#define F32XMERA_COB_CONVERTER_H

#include <architecture/messaging/messaging.h>

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
enum class PhaseAngleCorrectionMethod { NoCorrection, Lambertian, Binary };

const std::map<PhaseAngleCorrectionMethod, PhaseAngleCorrectionMethodAlgorithm> enumMap = {
    {PhaseAngleCorrectionMethod::NoCorrection, PhaseAngleCorrectionMethodAlgorithm::NoCorrectionAlg},
    {PhaseAngleCorrectionMethod::Lambertian, PhaseAngleCorrectionMethodAlgorithm::LambertianAlg},
    {PhaseAngleCorrectionMethod::Binary, PhaseAngleCorrectionMethodAlgorithm::BinaryAlg}};

/**
 * @class CobConverter
 * @brief Converts center-of-brightness (COB) pixel measurements into unit vectors
 *        (camera, body, inertial frames), with optional phase-angle correction
 *        and outlier detection.
 */
class CobConverter : public SysModel {
   public:
    CobConverter(PhaseAngleCorrectionMethod method, float radiusObject);
    ~CobConverter() final;

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

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
    void setOutlierDetectionEnabled(bool enable);
    bool isOutlierDetectionEnabled() const;
    void setBrownConradyCoefficients(const CalibrationCoefficients& coefficients);
    CalibrationCoefficients getBrownConradyCoefficients() const;

   public:
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
    CobConverterAlgorithm algorithm;
};

#endif  // F32XMERA_COB_CONVERTER_H
