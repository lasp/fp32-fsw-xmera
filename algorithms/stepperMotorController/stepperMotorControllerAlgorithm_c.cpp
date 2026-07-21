#include "stepperMotorControllerAlgorithm_c.h"
#include "stepperMotorControllerAlgorithm.h"
#include "stepperMotorControllerTypes.h"
#include "utilities/fsw/opaqueHandle.h"

namespace {
StepperMotorControllerConfig configFromC(const StepperMotorControllerConfig_c& c) {
    return StepperMotorControllerConfig::create(c.stepAngle,
                                                StepperMotorAngleRange{c.angleRange.minAngle, c.angleRange.maxAngle},
                                                c.settleCountMax,
                                                c.minStepCommand);
}
}  // namespace

StepperMotorControllerAlgorithmHandle* StepperMotorControllerAlgorithm_create(
    const StepperMotorControllerConfig_c* config) {
    return fsw::createHandle<::StepperMotorControllerAlgorithm, StepperMotorControllerAlgorithmHandle>(
        configFromC(*config));
}

void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithmHandle* self) {
    fsw::deleteHandle<::StepperMotorControllerAlgorithm>(self);
}

void StepperMotorControllerAlgorithm_setConfig(StepperMotorControllerAlgorithmHandle* self,
                                               const StepperMotorControllerConfig_c* config) {
    fsw::fromHandle<::StepperMotorControllerAlgorithm>(self)->setConfig(configFromC(*config));
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
