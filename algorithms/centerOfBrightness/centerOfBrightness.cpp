#include "centerOfBrightness.h"

inline constexpr double kNanoToSec = 1.0e-9;

/*! Module constructor */
CenterOfBrightness::CenterOfBrightness(std::shared_ptr<ImageReaderInterface> imageReaderInstance)
    : imageReader(std::move(imageReaderInstance)) {}

/*! Module destructor */
CenterOfBrightness::~CenterOfBrightness() = default;

/*! This method performs a complete reset of the module.  Local module variables that retain time varying states
 * between function calls are reset to their default values.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::reset(const uint64_t currentSimNanos) {
    if (!this->roiInMsg.isLinked()) {
        throw std::invalid_argument("CenterOfBrightness.roiInMsg wasn't connected.");
    }
    this->algorithm.reset();
    this->previousImageTimeTag = 0;
}

/*! This module reads a region of interest message and delegates image reading and center of brightness
 * computation to the algorithm and image reader.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void CenterOfBrightness::updateState(const uint64_t currentSimNanos) {
    const auto roiPayload = this->roiInMsg();
    OpNavCOBMsgPayload cobBuffer{};
    CenterOfBrightnessResult result{};

    const int64_t imageTimeTag =
        this->imageReader->getCurrentImageTimeTag(this->cameraID, static_cast<int64_t>(currentSimNanos * kNanoToSec));
    if (imageTimeTag > this->previousImageTimeTag) {
        this->previousImageTimeTag = imageTimeTag;

        const CobRegionOfInterest roi{Eigen::Vector2i(roiPayload.centerX, roiPayload.centerY),
                                      Eigen::Vector2i(roiPayload.width, roiPayload.height)};

        result = this->algorithm.update(roi, *this->imageReader);
    }

    cobBuffer.valid = result.valid;
    cobBuffer.centerOfBrightness[0] = result.centerOfBrightness[0];
    cobBuffer.centerOfBrightness[1] = result.centerOfBrightness[1];
    cobBuffer.pixelsFound = result.pixelsFound;
    cobBuffer.rollingAverageBrightness = result.rollingAverageBrightness;
    if (result.valid) {
        cobBuffer.timeTag = static_cast<uint64_t>(imageTimeTag);
        cobBuffer.cameraID = this->cameraID;
    }

    const CenterOfBrightnessDiagnosticMsgPayload diagnosticBuffer{result.noPixelTrigger,
                                                                  result.notExceedingBrightnessIncreaseTrigger};

    this->opnavCOBOutMsg.write(cobBuffer, this->moduleID, currentSimNanos);
    this->centerOfBrightnessDiagnosticOutMsg.write(diagnosticBuffer, this->moduleID, currentSimNanos);
}

/*! Delegating setters/getters for algorithm parameters */

void CenterOfBrightness::setRelativeBrightnessIncreaseThreshold(const double increaseThreshold) {
    this->algorithm.setRelativeBrightnessIncreaseThreshold(increaseThreshold);
}

double CenterOfBrightness::getRelativeBrightnessIncreaseThreshold() const {
    return this->algorithm.getRelativeBrightnessIncreaseThreshold();
}

void CenterOfBrightness::setNumberOfPointsBrightnessAverage(const int32_t rollingAverage) {
    this->algorithm.setNumberOfPointsBrightnessAverage(rollingAverage);
}

int32_t CenterOfBrightness::getNumberOfPointsBrightnessAverage() const {
    return this->algorithm.getNumberOfPointsBrightnessAverage();
}

/*! Adapter-only setters/getters */

void CenterOfBrightness::setCameraID(const int32_t id) { this->cameraID = id; }

int32_t CenterOfBrightness::getCameraID() const { return this->cameraID; }
