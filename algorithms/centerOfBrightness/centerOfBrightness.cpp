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

        // If no image is available, getImage gives an empty image. An empty image gives a null view, and the
        // algorithm then sets noPixelTrigger.
        const cv::Mat image = this->imageReader->getImage();
        const ImageView_c imageView{.pixels = image.empty() ? nullptr : image.ptr<uint8_t>(0),
                                    .rows = image.rows,
                                    .cols = image.cols,
                                    .strideBytes = static_cast<int32_t>(image.step[0])};
        const RegionOfInterest_c roi{.centerX = roiPayload.centerX,
                                     .centerY = roiPayload.centerY,
                                     .width = roiPayload.width,
                                     .height = roiPayload.height};

        result = this->algorithm->update(imageView, roi);
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
