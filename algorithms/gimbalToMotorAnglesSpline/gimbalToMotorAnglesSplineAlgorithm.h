#ifndef F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_ALGORITHM_H
#define F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_ALGORITHM_H

#include <math.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <numbers>

#include "gimbalToMotorAnglesSplineTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

static constexpr uint32_t kNumSplineIntervals = NUM_SPLINE_INTERVALS;
static constexpr uint32_t kNumSplineCoeffsPerAxis = NUM_SPLINE_COEFFS_PER_AXIS;

/*! @brief Structure containing the algorithm output. */
struct GimbalToMotorAnglesSplineOutput {
    float motorAngle1{};  //!< [rad] Motor 1 angle
    float motorAngle2{};  //!< [rad] Motor 2 angle
};

/*! @brief Addressable gimbal rhombus |u| <= 1, |v| <= 1, where
 * u = (tip - centerTipAngle) / halfDiagonalTip + (tilt - centerTiltAngle) / halfDiagonalTilt and
 * v = (tip - centerTipAngle) / halfDiagonalTip - (tilt - centerTiltAngle) / halfDiagonalTilt. */
struct GimbalRhombusDomain {
    float centerTipAngle;    //!< [rad] Gimbal tip angle at the rhombus center
    float centerTiltAngle;   //!< [rad] Gimbal tilt angle at the rhombus center
    float halfDiagonalTip;   //!< [rad] Half of the rhombus diagonal along the gimbal tip axis
    float halfDiagonalTilt;  //!< [rad] Half of the rhombus diagonal along the gimbal tilt axis
};

/*! @brief Type alias for the uniform bicubic B-spline control coefficients [steps] of one motor surface. Coefficient
 * c[i][j] is stored at index i * NUM_SPLINE_COEFFS_PER_AXIS + j, where i is along u and j is along v. */
using SplineCoefficients = std::array<float, NUM_SPLINE_COEFFS>;

static_assert(SPLINE_DEGREE == 3, "evaluateSurface implements cubic B-splines only");

/*! @brief Validated configuration for the gimbal to motor angles spline algorithm.
 * Holds the motor step angle, the default motor angles, the addressable gimbal rhombus, and the control coefficients
 * of the motor 1 and motor 2 step surfaces. */
class GimbalToMotorAnglesSplineConfig final {
   public:
    /// Same step angle bounds as stepperMotorController, so every accepted step angle is also accepted there.
    static constexpr uint32_t kMaxStepsPerRev = 100000U;
    static constexpr float kMinStepAngle = 2.0F * std::numbers::pi_v<float> / static_cast<float>(kMaxStepsPerRev);

    static GimbalToMotorAnglesSplineConfig create(float stepAngle,
                                                  float defaultMotor1Angle,
                                                  float defaultMotor2Angle,
                                                  const GimbalRhombusDomain& domain,
                                                  const SplineCoefficients& motor1Coefficients,
                                                  const SplineCoefficients& motor2Coefficients) {
        if (!isValidStepAngle(stepAngle)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalToMotorAnglesSpline: stepAngle must be in [2*pi/kMaxStepsPerRev, 2*pi].");
        }
        if (!isValidDefaultMotorAngle(defaultMotor1Angle)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalToMotorAnglesSpline: defaultMotor1Angle must be in [-pi, pi].");
        }
        if (!isValidDefaultMotorAngle(defaultMotor2Angle)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalToMotorAnglesSpline: defaultMotor2Angle must be in [-pi, pi].");
        }
        if (!isValidDomain(domain)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalToMotorAnglesSpline: the rhombus values must be finite, the half-diagonals must be greater "
                "than zero, and the rhombus must stay within +-pi/2 on each gimbal axis.");
        }
        if (!isValidCoefficients(motor1Coefficients)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalToMotorAnglesSpline: all motor 1 spline coefficients must be in [-kMaxStepsPerRev, "
                "kMaxStepsPerRev].");
        }
        if (!isValidCoefficients(motor2Coefficients)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalToMotorAnglesSpline: all motor 2 spline coefficients must be in [-kMaxStepsPerRev, "
                "kMaxStepsPerRev].");
        }

        return {stepAngle, defaultMotor1Angle, defaultMotor2Angle, domain, motor1Coefficients, motor2Coefficients};
    }

    /*! Checks that the step angle is in [2*pi/kMaxStepsPerRev, 2*pi]. NaN fails both comparisons.
     @return bool
     @param stepAngle [rad/step] Angle per motor step
    */
    static bool isValidStepAngle(float stepAngle) {
        return stepAngle >= kMinStepAngle && stepAngle <= 2.0F * std::numbers::pi_v<float>;
    }

    /*! Checks that a default motor angle is in [-pi, pi]. NaN fails both comparisons.
     @return bool
     @param defaultMotorAngle [rad] Default motor angle
    */
    static bool isValidDefaultMotorAngle(float defaultMotorAngle) {
        return defaultMotorAngle >= -std::numbers::pi_v<float> && defaultMotorAngle <= std::numbers::pi_v<float>;
    }

    /*! Checks that the rhombus values are finite, the half-diagonals are greater than zero, and the rhombus stays
     within the +-90 degree gimbal limits on each axis: |center| + half-diagonal <= pi/2.
     @return bool
     @param domain [rad] Addressable gimbal rhombus
    */
    static bool isValidDomain(const GimbalRhombusDomain& domain) {
        if (!fsw::is_finite(domain.centerTipAngle) || !fsw::is_finite(domain.centerTiltAngle) ||
            !fsw::is_finite(domain.halfDiagonalTip) || !fsw::is_finite(domain.halfDiagonalTilt)) {
            return false;
        }
        if (domain.halfDiagonalTip <= 0.0F || domain.halfDiagonalTilt <= 0.0F) {
            return false;
        }
        constexpr float maxGimbalAngle = std::numbers::pi_v<float> / 2.0F;  // [rad]
        return fabsf(domain.centerTipAngle) + domain.halfDiagonalTip <= maxGimbalAngle &&
               fabsf(domain.centerTiltAngle) + domain.halfDiagonalTilt <= maxGimbalAngle;
    }

    /*! Checks that every spline coefficient is in [-kMaxStepsPerRev, kMaxStepsPerRev]. The surface lies in the
     coefficient convex hull, so every surface value is also in this range. NaN fails both comparisons.
     @return bool
     @param coefficients [steps] Spline control coefficients
    */
    static bool isValidCoefficients(const SplineCoefficients& coefficients) {
        return std::ranges::all_of(coefficients,
                                   [](float value) { return fabsf(value) <= static_cast<float>(kMaxStepsPerRev); });
    }

    float getStepAngle() const { return this->stepAngle; }
    float getDefaultMotor1Angle() const { return this->defaultMotor1Angle; }
    float getDefaultMotor2Angle() const { return this->defaultMotor2Angle; }
    const GimbalRhombusDomain& getDomain() const { return this->domain; }
    const SplineCoefficients& getMotor1Coefficients() const { return this->motor1Coefficients; }
    const SplineCoefficients& getMotor2Coefficients() const { return this->motor2Coefficients; }

   private:
    GimbalToMotorAnglesSplineConfig(float stepAngle,
                                    float defaultMotor1Angle,
                                    float defaultMotor2Angle,
                                    const GimbalRhombusDomain& domain,
                                    const SplineCoefficients& motor1Coefficients,
                                    const SplineCoefficients& motor2Coefficients)
        : stepAngle(stepAngle),
          defaultMotor1Angle(defaultMotor1Angle),
          defaultMotor2Angle(defaultMotor2Angle),
          domain(domain),
          motor1Coefficients(motor1Coefficients),
          motor2Coefficients(motor2Coefficients) {}

    float stepAngle;                        //!< [rad/step] Angle per motor step
    float defaultMotor1Angle;               //!< [rad] Motor 1 angle output before the first valid request
    float defaultMotor2Angle;               //!< [rad] Motor 2 angle output before the first valid request
    GimbalRhombusDomain domain;             //!< [rad] Addressable gimbal rhombus
    SplineCoefficients motor1Coefficients;  //!< [steps] Motor 1 step surface control coefficients
    SplineCoefficients motor2Coefficients;  //!< [steps] Motor 2 step surface control coefficients
};

/*! @brief Pure algorithm: determines the stepper motor angles corresponding to the incoming gimbal tip and tilt
 * angles by evaluating one uniform bicubic B-spline surface for each motor. Motor 1 is S1(u, v) and motor 2 is
 * S2(u, v). */
class GimbalToMotorAnglesSplineAlgorithm final {
   public:
    /// [-] Allowed overshoot of |u| and |v| past 1 so that fp32 rounding does not reject the rhombus boundary.
    /// u is about 5 operations on inputs that are each rounded once (about 9 epsilon for |u| <= 1, ~1.1e-6);
    /// 1e-5 gives about 10x margin. 1e-5 in u is about 3e-6 rad of gimbal angle.
    static constexpr float kDomainTolerance = 1e-5F;

    explicit GimbalToMotorAnglesSplineAlgorithm(const GimbalToMotorAnglesSplineConfig& config);
    void setConfig(const GimbalToMotorAnglesSplineConfig& config);
    void reInitialize();
    GimbalToMotorAnglesSplineOutput update(float gimbalAngle1, float gimbalAngle2);

   private:
    GimbalToMotorAnglesSplineOutput previousValidOutput{};  //!< [rad] Last valid algorithm output
    GimbalToMotorAnglesSplineConfig cfg;
};

#endif /* F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_ALGORITHM_H */
