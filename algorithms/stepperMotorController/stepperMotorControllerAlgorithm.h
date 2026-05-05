#ifndef F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H
#define F32XIMERA_STEPPER_MOTOR_CONTROLLER_ALGORITHM_H

#include "stepperMotorControllerTypes.h"
#include <array>
#include <cstdint>
#include <numbers>

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
class StepperMotorControllerAlgorithm {
   public:
    void reset();
    StepperMotorControllerOutput update(int currentPosition, float referenceAngle, bool isMotorMoving);

    void setStepAngle(float stepAngleIn);
    float getStepAngle() const;
    void setMotorAngleRange(float minAngleIn, float maxAngleIn);
    std::array<float, 2> getMotorAngleRange() const;
    void setSettleCountMax(uint32_t settleCountMaxIn);
    uint32_t getSettleCountMax() const;
    void setCurrentPositionTolerance(uint32_t currentPositionToleranceIn);
    uint32_t getCurrentPositionTolerance() const;
    void setDesiredPositionTolerance(uint32_t desiredPositionToleranceIn);
    uint32_t getDesiredPositionTolerance() const;

    int angleToSteps(float angle) const;

   private:
    int wrapDelta(int delta) const;
    int stepDelta(int delta) const;

    /// Cap on stepsPerRev so the fp32 round-trip in angleToSteps stays within rounding tolerance
    /// (error grows as O(N^2 * eps); at N=100k the expected drift is ~0.19 steps, safely within
    /// the half-step round() margin).
    static constexpr uint32_t kMaxStepsPerRev = 100000U;
    static constexpr float kMinStepAngle = 2.0F * std::numbers::pi_v<float> / static_cast<float>(kMaxStepsPerRev);

    // Parameters
    float stepAngle{};     //!< [rad/step] Angle per motor step (must be set via setStepAngle)
    int stepsPerRev{};     //!< [steps] Derived: round(2*pi / stepAngle), cached by setStepAngle for wrap math
    float minAngle{0.0F};  //!< [rad] Lower bound of motor travel range
    float maxAngle{2.0F * std::numbers::pi_v<float>};  //!< [rad] Upper bound of motor travel range
    bool isFullCircle{true};               //!< Derived: (maxAngle - minAngle) within eps of 2*pi; enables wrap-around
    uint32_t settleCountMax{10};           //!< [ticks] Settling duration after stop
    uint32_t currentPositionTolerance{1};  //!< [steps] Tolerance between current and target position (IDLE/STOPPING)
    uint32_t desiredPositionTolerance{
        0};  //!< [steps] Tolerance between commanded and desired position (MOVING interrupt check)

    // State
    StepperMotorState state{StepperMotorState::IDLE};
    int commandedPosition{};  //!< [steps] Target of the current move command
    int desiredPosition{};    //!< [steps] Desired position from reference angle
    uint32_t settleCount{};   //!< [ticks] Counter for settling phase
};

#endif
