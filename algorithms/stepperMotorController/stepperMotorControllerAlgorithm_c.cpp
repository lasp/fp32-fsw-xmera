#include "stepperMotorControllerAlgorithm_c.h"
#include "stepperMotorControllerAlgorithm.h"
#include "stepperMotorControllerTypes.h"
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
