#include "stepperMotorControllerAlgorithm_c.h"
#include "stepperMotorControllerAlgorithm.h"

StepperMotorControllerAlgorithmHandle* StepperMotorControllerAlgorithm_create(void) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<StepperMotorControllerAlgorithmHandle*>(new ::StepperMotorControllerAlgorithm());
}

void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<::StepperMotorControllerAlgorithm*>(self);
}

void StepperMotorControllerAlgorithm_reset(StepperMotorControllerAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->reset();
}

StepperMotorControllerOutput StepperMotorControllerAlgorithm_update(StepperMotorControllerAlgorithmHandle* self,
                                                                    const int32_t currentPosition,
                                                                    const float referenceAngle,
                                                                    const bool isMotorMoving) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->update(
        currentPosition, referenceAngle, isMotorMoving);
}

int32_t StepperMotorControllerAlgorithm_angleToSteps(const StepperMotorControllerAlgorithmHandle* self,
                                                     const float angle) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->angleToSteps(angle);
}

void StepperMotorControllerAlgorithm_setStepAngle(StepperMotorControllerAlgorithmHandle* self, const float stepAngle) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setStepAngle(stepAngle);
}

float StepperMotorControllerAlgorithm_getStepAngle(const StepperMotorControllerAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getStepAngle();
}

void StepperMotorControllerAlgorithm_setMotorAngleRange(StepperMotorControllerAlgorithmHandle* self,
                                                        const float minAngle,
                                                        const float maxAngle) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setMotorAngleRange(minAngle, maxAngle);
}

MotorAngleRange_c StepperMotorControllerAlgorithm_getMotorAngleRange(
    const StepperMotorControllerAlgorithmHandle* self) {
    const std::array<float, 2> range =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getMotorAngleRange();
    return {range[0], range[1]};
}

void StepperMotorControllerAlgorithm_setSettleCountMax(StepperMotorControllerAlgorithmHandle* self,
                                                       const uint32_t settleCountMax) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setSettleCountMax(settleCountMax);
}

uint32_t StepperMotorControllerAlgorithm_getSettleCountMax(const StepperMotorControllerAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getSettleCountMax();
}

void StepperMotorControllerAlgorithm_setMinStepCommand(StepperMotorControllerAlgorithmHandle* self,
                                                       const uint32_t minStepCommand) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setMinStepCommand(minStepCommand);
}

uint32_t StepperMotorControllerAlgorithm_getMinStepCommand(const StepperMotorControllerAlgorithmHandle* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getMinStepCommand();
}
