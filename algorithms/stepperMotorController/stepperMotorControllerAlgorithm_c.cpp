#include "stepperMotorControllerAlgorithm_c.h"
#include "stepperMotorControllerAlgorithm.h"

StepperMotorControllerAlgorithm* StepperMotorControllerAlgorithm_create(void) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<StepperMotorControllerAlgorithm*>(new ::StepperMotorControllerAlgorithm());
}

void StepperMotorControllerAlgorithm_destroy(StepperMotorControllerAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-owning-memory)
    delete reinterpret_cast<::StepperMotorControllerAlgorithm*>(self);
}

void StepperMotorControllerAlgorithm_reset(StepperMotorControllerAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->reset();
}

StepperMotorControllerOutput StepperMotorControllerAlgorithm_update(StepperMotorControllerAlgorithm* self,
                                                                    const int32_t currentPosition,
                                                                    const float referenceAngle,
                                                                    const bool isMotorMoving) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->update(
        currentPosition, referenceAngle, isMotorMoving);
}

int32_t StepperMotorControllerAlgorithm_angleToSteps(const StepperMotorControllerAlgorithm* self, const float angle) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->angleToSteps(angle);
}

void StepperMotorControllerAlgorithm_setStepAngle(StepperMotorControllerAlgorithm* self, const float stepAngle) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setStepAngle(stepAngle);
}

float StepperMotorControllerAlgorithm_getStepAngle(const StepperMotorControllerAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getStepAngle();
}

void StepperMotorControllerAlgorithm_setMotorAngleRange(StepperMotorControllerAlgorithm* self,
                                                        const float minAngle,
                                                        const float maxAngle) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setMotorAngleRange(minAngle, maxAngle);
}

MotorAngleRange_c StepperMotorControllerAlgorithm_getMotorAngleRange(const StepperMotorControllerAlgorithm* self) {
    const std::array<float, 2> range =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getMotorAngleRange();
    return {range[0], range[1]};
}

void StepperMotorControllerAlgorithm_setSettleCountMax(StepperMotorControllerAlgorithm* self,
                                                       const uint32_t settleCountMax) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setSettleCountMax(settleCountMax);
}

uint32_t StepperMotorControllerAlgorithm_getSettleCountMax(const StepperMotorControllerAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getSettleCountMax();
}

void StepperMotorControllerAlgorithm_setMinStepCommand(StepperMotorControllerAlgorithm* self,
                                                       const uint32_t minStepCommand) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    reinterpret_cast<::StepperMotorControllerAlgorithm*>(self)->setMinStepCommand(minStepCommand);
}

uint32_t StepperMotorControllerAlgorithm_getMinStepCommand(const StepperMotorControllerAlgorithm* self) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const ::StepperMotorControllerAlgorithm*>(self)->getMinStepCommand();
}
