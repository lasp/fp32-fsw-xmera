#include "stepperMotorControllerAlgorithm_c.h"
#include "stepperMotorControllerAlgorithm.h"
#include "stepperMotorControllerTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/opaqueHandle.h"

namespace {
StepperMotorControllerConfig configFromC(const float stepAngle,
                                         const float minAngle,
                                         const float maxAngle,
                                         const uint32_t settleCountMax,
                                         const uint32_t minStepCommand) {
    return StepperMotorControllerConfig::create(
        stepAngle, StepperMotorAngleRange{minAngle, maxAngle}, settleCountMax, minStepCommand);
}
}  // namespace

bool StepperMotorControllerAlgorithm_validateConfig(const float stepAngle,
                                                    const float minAngle,
                                                    const float maxAngle,
                                                    const uint32_t settleCountMax,
                                                    const uint32_t minStepCommand) {
    // Attempt to build the config through the real create path; success means valid,
    // a throw means invalid. Reusing configFromC keeps validation from drifting.
    try {
        (void)configFromC(stepAngle, minAngle, maxAngle, settleCountMax, minStepCommand);
        return true;
    } catch (const fsw::invalid_argument&) {
        return false;
    }
}

StepperMotorControllerAlgorithmHandle* StepperMotorControllerAlgorithm_create(const float stepAngle,
                                                                              const float minAngle,
                                                                              const float maxAngle,
                                                                              const uint32_t settleCountMax,
                                                                              const uint32_t minStepCommand) {
    return fsw::createHandle<::StepperMotorControllerAlgorithm, StepperMotorControllerAlgorithmHandle>(
        configFromC(stepAngle, minAngle, maxAngle, settleCountMax, minStepCommand));
}

void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithmHandle* self) {
    fsw::deleteHandle<::StepperMotorControllerAlgorithm>(self);
}

void StepperMotorControllerAlgorithm_setConfig(StepperMotorControllerAlgorithmHandle* self,
                                               const float stepAngle,
                                               const float minAngle,
                                               const float maxAngle,
                                               const uint32_t settleCountMax,
                                               const uint32_t minStepCommand) {
    fsw::fromHandle<::StepperMotorControllerAlgorithm>(self)->setConfig(
        configFromC(stepAngle, minAngle, maxAngle, settleCountMax, minStepCommand));
}

void StepperMotorControllerAlgorithm_reInitialize(StepperMotorControllerAlgorithmHandle* self) {
    fsw::fromHandle<::StepperMotorControllerAlgorithm>(self)->reInitialize();
}

StepperMotorControllerOutput StepperMotorControllerAlgorithm_update(StepperMotorControllerAlgorithmHandle* self,
                                                                    const int32_t currentPosition,
                                                                    const float referenceAngle,
                                                                    const bool isMotorMoving) {
    return fsw::fromHandle<::StepperMotorControllerAlgorithm>(self)->update(
        currentPosition, referenceAngle, isMotorMoving);
}

int32_t StepperMotorControllerAlgorithm_angleToSteps(const StepperMotorControllerAlgorithmHandle* self,
                                                     const float angle) {
    return fsw::fromHandle<const ::StepperMotorControllerAlgorithm>(self)->angleToSteps(angle);
}
