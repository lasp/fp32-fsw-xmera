#include "centerOfBrightness.h"
#include "utilities/xmera/xmeraLifecycleException.h"

inline constexpr double kNanoToSec = 1.0e-9;

/*! Module constructor */
CenterOfBrightness::CenterOfBrightness(std::shared_ptr<ImageReaderInterface> imageReaderInstance)
    : imageReader(std::move(imageReaderInstance)) {}

/*! Module destructor */
CenterOfBrightness::~CenterOfBrightness() = default;

/*! This method builds the validated configuration and constructs the algorithm. Local module variables
 * that retain time varying states between function calls are reset to their default values.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::reset(const uint64_t currentSimNanos) {
    if (!this->roiInMsg.isLinked()) {
        throw std::invalid_argument("CenterOfBrightness.roiInMsg wasn't connected.");
    }
    auto config = CenterOfBrightnessConfig::create(this->relativeBrightnessIncreaseThreshold,
                                                   this->numberOfPointsBrightnessAverage);
    this->algorithm = std::make_unique<CenterOfBrightnessAlgorithm>(config);
    this->previousImageTimeTag = 0;
}

/*! This module reads a region of interest message and delegates image reading and center of brightness
 * computation to the algorithm and image reader.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::updateState(const uint64_t currentSimNanos) {
    if (!this->algorithm) {
        throw XmeraLifecycleException("CenterOfBrightness reset() has not been called.");
    }

    const auto roiPayload = this->roiInMsg();
    OpNavCOBMsgF32Payload cobBuffer{};
    CenterOfBrightnessResult result{};

    const int64_t imageTimeTag =
        this->imageReader->getCurrentImageTimeTag(this->cameraID, static_cast<int64_t>(currentSimNanos * kNanoToSec));
    if (imageTimeTag > this->previousImageTimeTag) {
        this->previousImageTimeTag = imageTimeTag;

        const Eigen::Vector2i roiCenter(roiPayload.centerX, roiPayload.centerY);
        const Eigen::Vector2i roiSize(roiPayload.width, roiPayload.height);
        this->imageReader->getImageAsArray(roiCenter, roiSize, *this->pixelBuffer);

        result = this->algorithm->update(*this->pixelBuffer);
    }

    cobBuffer.valid = result.valid;
    cobBuffer.centerOfBrightness[0] = result.centerOfBrightness[0];
    cobBuffer.centerOfBrightness[1] = result.centerOfBrightness[1];
    cobBuffer.pixelsFound = result.pixelsFound;
    cobBuffer.rollingAverageBrightness = result.rollingAverageBrightness;
    if (result.valid) {
        cobBuffer.timeTag = static_cast<uint64_t>(imageTimeTag);
    }

    const CenterOfBrightnessDiagnosticMsgF32Payload diagnosticBuffer{result.noPixelTrigger,
                                                                     result.notExceedingBrightnessIncreaseTrigger};

    this->opnavCOBOutMsg.write(cobBuffer, this->moduleID, currentSimNanos);
    this->centerOfBrightnessDiagnosticOutMsg.write(diagnosticBuffer, this->moduleID, currentSimNanos);
}
