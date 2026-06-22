#include "stepperMotorControllerAlgorithm_c.h"
#include "stepperMotorControllerAlgorithm.h"
#include "stepperMotorControllerTypes.h"

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
    return reinterpret_cast<StepperMotorControllerAlgorithmHandle*>(
        new ::StepperMotorControllerAlgorithm(configFromC(*config)));
}

void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithmHandle* self) {
    delete reinterpret_cast<::StepperMotorControllerAlgorithm*>(self);
}

void StepperMotorControllerAlgorithm_setConfig(StepperMotorControllerAlgorithmHandle* self,
                                               const StepperMotorControllerConfig_c* config) {
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setConfig(configFromC(*config));
}

void StepperMotorControllerAlgorithm_reInitialize(StepperMotorControllerAlgorithmHandle* self) {
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->reInitialize();
}

StepperMotorControllerOutput StepperMotorControllerAlgorithm_update(StepperMotorControllerAlgorithmHandle* self,
                                                                    const int32_t currentPosition,
                                                                    const float referenceAngle,
                                                                    const bool isMotorMoving) {
    return reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->update(
        currentPosition, referenceAngle, isMotorMoving);
}

int32_t StepperMotorControllerAlgorithm_angleToSteps(const StepperMotorControllerAlgorithmHandle* self,
                                                     const float angle) {
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->angleToSteps(angle);
}
