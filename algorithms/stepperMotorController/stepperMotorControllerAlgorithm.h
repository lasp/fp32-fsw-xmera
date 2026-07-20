#ifndef F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H
#define F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H

#include "stepperMotorControllerTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include <math.h>
#include <cstdint>
#include <numbers>

/*! @brief Motor angular travel range in body-frame radians. */
struct StepperMotorAngleRange {
    float minAngle{0.0F};                              //!< [rad] lower bound of the motor travel range
    float maxAngle{2.0F * std::numbers::pi_v<float>};  //!< [rad] upper bound of the motor travel range
};

/*!
 * @brief Validated configuration for the stepper motor controller algorithm.
 *
 * An instance can only exist with a step angle in [2*pi/kMaxStepsPerRev, 2*pi]; an angle range whose bounds are in
 * [-2*pi, 2*pi] with minAngle strictly less than maxAngle; and a strictly positive minimum step command. The steps
 * per revolution and the full-circle flag are derived from the step angle and range when the configuration is built.
 * Construct via StepperMotorControllerConfig::create(...).
 */
class StepperMotorControllerConfig final {
   public:
    /// Cap on stepsPerRev so the fp32 round-trip in angleToSteps stays within rounding tolerance
    /// (error grows as O(N^2 * eps); at N=100k the expected drift is ~0.19 steps, safely within
    /// the half-step round() margin).
    static constexpr uint32_t kMaxStepsPerRev = 100000U;
    static constexpr float kMinStepAngle = 2.0F * std::numbers::pi_v<float> / static_cast<float>(kMaxStepsPerRev);

    static StepperMotorControllerConfig create(float stepAngle,
                                               const StepperMotorAngleRange& angleRange,
                                               uint32_t settleCountMax,
                                               uint32_t minStepCommand) {
        if (!isValidStepAngle(stepAngle)) {
            FSW_THROW_INVALID_ARGUMENT("stepperMotorController: stepAngle must be in [2*pi/kMaxStepsPerRev, 2*pi].");
        }
        if (!isValidAngleRange(angleRange)) {
            FSW_THROW_INVALID_ARGUMENT(
                "stepperMotorController: minAngle and maxAngle must be in [-2*pi, 2*pi] with minAngle strictly less "
                "than maxAngle.");
        }
        if (!isValidMinStepCommand(minStepCommand)) {
            FSW_THROW_INVALID_ARGUMENT("stepperMotorController: minStepCommand must be greater than zero.");
        }
        return {stepAngle, angleRange, settleCountMax, minStepCommand};
    }

    static bool isValidStepAngle(float stepAngle) {
        constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
        return stepAngle >= kMinStepAngle && stepAngle <= twoPi;
    }

    static bool isValidAngleRange(const StepperMotorAngleRange& angleRange) {
        constexpr float twoPi = 2.0F * std::numbers::pi_v<float>;
        return angleRange.minAngle >= -twoPi && angleRange.minAngle <= twoPi && angleRange.maxAngle >= -twoPi &&
               angleRange.maxAngle <= twoPi && angleRange.minAngle < angleRange.maxAngle;
    }

    static bool isValidMinStepCommand(uint32_t minStepCommand) { return minStepCommand > 0U; }
    // No isValidSettleCountMax -- any uint32_t tick count is valid.

    float getStepAngle() const { return this->stepAngle; }
    const StepperMotorAngleRange& getAngleRange() const { return this->angleRange; }
    uint32_t getSettleCountMax() const { return this->settleCountMax; }
    uint32_t getMinStepCommand() const { return this->minStepCommand; }

   private:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters): settleCountMax (ticks) and minStepCommand (steps) are
    // distinct documented quantities stored verbatim by the factory.
    StepperMotorControllerConfig(float stepAngle,
                                 const StepperMotorAngleRange& angleRange,
                                 uint32_t settleCountMax,
                                 uint32_t minStepCommand)
        : stepAngle(stepAngle),
          angleRange(angleRange),
          settleCountMax(settleCountMax),
          minStepCommand(minStepCommand) {}
    // NOLINTEND(bugprone-easily-swappable-parameters)

    float stepAngle;                    //!< [rad/step] angle per motor step
    StepperMotorAngleRange angleRange;  //!< [rad] motor travel range
    uint32_t settleCountMax;            //!< [ticks] settling duration after stop
    uint32_t minStepCommand;            //!< [steps] minimum step delta magnitude that triggers a command (> 0)
};

/*! @brief Stepper Motor Controller Algorithm
 *
 * State-machine-based controller that commands a stepper motor to a desired position.
 * Operates in integer step counts internally; reference angles are converted using stepAngle.
 * The current motor position is an input to update(); position tracking lives in the caller.
 *
 * The motor's travel range is bounded by [minAngle, maxAngle]. Reference angles outside this
 * range are rejected (no movement). When the range covers the full circle the controller uses
 * shortest-path wrap-around; for a partial range it uses a linear delta so the motor cannot
 * cross the out-of-range boundary.
 */
class StepperMotorControllerAlgorithm final {
   public:
    explicit StepperMotorControllerAlgorithm(const StepperMotorControllerConfig& config);

    void setConfig(const StepperMotorControllerConfig& config);

    //! Reset the controller state machine to IDLE and clear cached positions.
    void reInitialize();

    StepperMotorControllerOutput update(int currentPosition, float referenceAngle, bool isMotorMoving);

    int angleToSteps(float angle) const;

   private:
    //! Recompute the cached values derived from the configuration (steps per revolution, full-circle flag).
    void cacheDerivedValues();
    int wrapDelta(int delta) const;
    int stepDelta(int delta) const;

    StepperMotorControllerConfig cfg;  //!< [-] validated configuration

    // Cached values derived from the configuration (recomputed on construction and setConfig).
    int stepsPerRev{};    //!< [steps] derived round(2*pi / stepAngle), used for wrap math
    bool isFullCircle{};  //!< derived: range within eps of 2*pi; enables shortest-path wrap-around

    // State
    StepperMotorState state{StepperMotorState::IDLE};
    int commandedPosition{};  //!< [steps] Target of the current move command
    int desiredPosition{};    //!< [steps] Desired position from reference angle
    uint32_t settleCount{};   //!< [ticks] Counter for settling phase
};

#endif
