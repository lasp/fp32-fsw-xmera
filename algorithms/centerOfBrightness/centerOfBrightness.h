#ifndef F32XMERA_CENTER_OF_BRIGHTNESS_H
#define F32XMERA_CENTER_OF_BRIGHTNESS_H

#include <architecture/messaging/messaging.h>
#include <stdint.h>
#include <memory>

#include "msgPayloadDef/CenterOfBrightnessDiagnosticMsgF32Payload.h"
#include "msgPayloadDef/OpNavCOBMsgF32Payload.h"
#include "msgPayloadDef/RegionOfInterestMsgF32Payload.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>

#include "centerOfBrightnessAlgorithm.h"
#include "imageReaderInterface.h"

/*! @brief visual object tracking using center of brightness detection */
class CenterOfBrightness : public SysModel {
   public:
    explicit CenterOfBrightness(std::shared_ptr<ImageReaderInterface> imageReaderInstance);
    ~CenterOfBrightness() override;

    void updateState(uint64_t currentSimNanos) override;
    void reset(uint64_t currentSimNanos) override;

    void setRelativeBrightnessIncreaseThreshold(float increaseThreshold);
    float getRelativeBrightnessIncreaseThreshold() const;
    void setNumberOfPointsBrightnessAverage(int32_t rollingAverage);
    int32_t getNumberOfPointsBrightnessAverage() const;
    void setCameraID(int32_t id);
    int32_t getCameraID() const;

    Message<OpNavCOBMsgF32Payload> opnavCOBOutMsg;  //!< The name of the OpNav center of brightness output message
    Message<CenterOfBrightnessDiagnosticMsgF32Payload> centerOfBrightnessDiagnosticOutMsg;
    ReadFunctor<RegionOfInterestMsgF32Payload> roiInMsg;  //!< Region of interest input message

   private:
    void rebuildAlgorithmConfig();

    CenterOfBrightnessAlgorithm algorithm{CenterOfBrightnessConfig::create(0.0F, 1)};
    std::shared_ptr<ImageReaderInterface> imageReader;  //!< shared ownership with Python/SWIG
    float relativeBrightnessIncreaseThreshold{0.0F};    //!< [-] minimum relative brightness increase
    int32_t numberOfPointsBrightnessAverage{1};  //!< [-] number of points to be used for rolling average of brightness
    int32_t cameraID{};
    int64_t previousImageTimeTag{};
};

#endif  // F32XMERA_CENTER_OF_BRIGHTNESS_H
