#include "centerOfBrightness.h"
#include "utilities/fsw/freestandingInvalidArgument.h"

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
    this->rebuildAlgorithmConfig();
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
    OpNavCOBMsgF32Payload cobBuffer{};
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
    }

    const CenterOfBrightnessDiagnosticMsgF32Payload diagnosticBuffer{result.noPixelTrigger,
                                                                     result.notExceedingBrightnessIncreaseTrigger};

    this->opnavCOBOutMsg.write(cobBuffer, this->moduleID, currentSimNanos);
    this->centerOfBrightnessDiagnosticOutMsg.write(diagnosticBuffer, this->moduleID, currentSimNanos);
}

/*! Delegating setters/getters for algorithm parameters */

void CenterOfBrightness::setRelativeBrightnessIncreaseThreshold(const float increaseThreshold) {
    if (!CenterOfBrightnessConfig::isValidRelativeBrightnessIncreaseThreshold(increaseThreshold)) {
        FSW_THROW_INVALID_ARGUMENT("centerOfBrightness: relativeBrightnessIncreaseThreshold must be non-negative.");
    }
    this->relativeBrightnessIncreaseThreshold = increaseThreshold;
    this->rebuildAlgorithmConfig();
}

float CenterOfBrightness::getRelativeBrightnessIncreaseThreshold() const {
    return this->relativeBrightnessIncreaseThreshold;
}

void CenterOfBrightness::setNumberOfPointsBrightnessAverage(const int32_t rollingAverage) {
    if (!CenterOfBrightnessConfig::isValidNumberOfPointsBrightnessAverage(rollingAverage)) {
        FSW_THROW_INVALID_ARGUMENT("centerOfBrightness: numberOfPointsBrightnessAverage must be positive.");
    }
    this->numberOfPointsBrightnessAverage = rollingAverage;
    this->rebuildAlgorithmConfig();
}

int32_t CenterOfBrightness::getNumberOfPointsBrightnessAverage() const { return this->numberOfPointsBrightnessAverage; }

void CenterOfBrightness::rebuildAlgorithmConfig() {
    const CenterOfBrightnessConfig config = CenterOfBrightnessConfig::create(this->relativeBrightnessIncreaseThreshold,
                                                                             this->numberOfPointsBrightnessAverage);
    this->algorithm.setConfig(config);
}

/*! Adapter-only setters/getters */

void CenterOfBrightness::setCameraID(const int32_t id) {
    if (id < 0) {
        FSW_THROW_INVALID_ARGUMENT("centerOfBrightness: cameraID must be non-negative.");
    }
    this->cameraID = id;
}

int32_t CenterOfBrightness::getCameraID() const { return this->cameraID; }
