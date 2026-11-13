#ifndef F32XMERA_GIMBAL_ANGLES_TO_MOTOR_ANGLES_ALGORITHM_H
#define F32XMERA_GIMBAL_ANGLES_TO_MOTOR_ANGLES_ALGORITHM_H

#include <math.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <numbers>
#include <optional>

#include "gimbalAnglesToMotorAnglesTypes.h"
#include "msgPayloadDef/definitions.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

static constexpr uint32_t kNumSplineIntervals = NUM_SPLINE_INTERVALS;
static constexpr uint32_t kNumSplineCoeffsPerAxis = NUM_SPLINE_COEFFS_PER_AXIS;

/*! @brief Structure containing the algorithm output. */
struct GimbalAnglesToMotorAnglesOutput {
    float motorAngle1{};  //!< [rad] Motor 1 angle
    float motorAngle2{};  //!< [rad] Motor 2 angle
};

/*! @brief Addressable gimbal rhombus |u| <= 1, |v| <= 1 centered at zero gimbal angles, where
 * u = tip / halfDiagonalTip + tilt / halfDiagonalTilt and v = tip / halfDiagonalTip - tilt / halfDiagonalTilt. */
struct GimbalRhombusDomain {
    float halfDiagonalTip;   //!< [rad] Half of the rhombus diagonal along the gimbal tip axis
    float halfDiagonalTilt;  //!< [rad] Half of the rhombus diagonal along the gimbal tilt axis
    float thetaMax;          //!< [rad] Maximum requested thrust cone angle. Must fit inside the rhombus.
};

/*! @brief Type alias for the uniform bicubic B-spline control coefficients [steps] of one motor surface. Coefficient
 * c[i][j] is stored at index i * NUM_SPLINE_COEFFS_PER_AXIS + j, where i is along u and j is along v. */
using SplineCoefficients = std::array<float, NUM_SPLINE_COEFFS>;

static_assert(SPLINE_DEGREE == 3, "evaluateSurface implements cubic B-splines only");

/*! @brief Spline evaluation of the algorithm. These functions are internal to the algorithm, and are declared here so
 * that the tests can check the spline directly, without the gimbal angle conversion and the step angle. */
namespace gimbal_angles_to_motor_angles_detail {

/*! @brief Spline interval that holds a normalized coordinate, and the local coordinate in it. */
struct SplineIntervalLocation {
    uint32_t interval{};  //!< [-] Interval index in [0, NUM_SPLINE_INTERVALS - 1]
    float t{};            //!< [-] Local coordinate in the interval, in [0, 1]
};

/*! This method computes the four uniform cubic B-spline weights for the local coordinate in a spline interval.
 @return std::array<float, 4> [-] Weights of the coefficients c_k to c_{k+3} of interval k
 @param t [-] Local coordinate in the interval, in [0, 1]
*/
std::array<float, 4> cubicBSplineWeights(float t);

/*! This method finds the spline interval that holds a normalized coordinate and the local coordinate in it. The
 interval is clamped at both ends, thus x = 1 is in the last interval with t = 1.
 @return SplineIntervalLocation
 @param x [-] Normalized coordinate in [-1, 1]
*/
SplineIntervalLocation splineInterval(float x);

/*! This method evaluates a uniform bicubic B-spline surface.
 @return float [steps] Surface value
 @param coefficients [steps] Control coefficients of the surface
 @param u [-] First normalized rhombus coordinate in [-1, 1]
 @param v [-] Second normalized rhombus coordinate in [-1, 1]
*/
float evaluateSurface(const SplineCoefficients& coefficients, float u, float v);

}  // namespace gimbal_angles_to_motor_angles_detail

/*! @brief Validated configuration for the gimbal to motor angles spline algorithm.
 * Holds the motor steps per revolution, the addressable gimbal rhombus, and the control coefficients of the motor 1
 * and motor 2 step surfaces. */
class GimbalAnglesToMotorAnglesConfig final {
   public:
    static GimbalAnglesToMotorAnglesConfig create(uint32_t stepsPerRevolution,
                                                  const GimbalRhombusDomain& domain,
                                                  const SplineCoefficients& motor1Coefficients,
                                                  const SplineCoefficients& motor2Coefficients) {
        if (!isValidStepsPerRevolution(stepsPerRevolution)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalAnglesToMotorAngles: stepsPerRevolution must be in [1, kMaxStepsPerRev].");
        }
        if (!isValidDomain(domain)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalAnglesToMotorAngles: the half-diagonals must be in (0, pi/2], and thetaMax must be greater "
                "than zero and fit inside the rhombus.");
        }
        if (!isValidCoefficients(motor1Coefficients)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalAnglesToMotorAngles: all motor 1 spline coefficients must be in [-kMaxStepsPerRev, "
                "kMaxStepsPerRev].");
        }
        if (!isValidCoefficients(motor2Coefficients)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalAnglesToMotorAngles: all motor 2 spline coefficients must be in [-kMaxStepsPerRev, "
                "kMaxStepsPerRev].");
        }

        return {stepsPerRevolution, domain, motor1Coefficients, motor2Coefficients};
    }

    /*! Checks that the motor steps per revolution are in [1, kMaxStepsPerRev].
     @return bool
     @param stepsPerRevolution [steps] Number of motor steps per 2*pi rad
    */
    static bool isValidStepsPerRevolution(uint32_t stepsPerRevolution) {
        return stepsPerRevolution >= 1U && stepsPerRevolution <= kMaxStepsPerRev;
    }

    /*! Computes the radius of the largest circle about zero gimbal angles inside the rhombus, a b / sqrt(a^2 + b^2),
     in fp32.
     @return float [rad]
     @param domain [rad] Addressable gimbal rhombus
    */
    static float inscribedRadius(const GimbalRhombusDomain& domain) {
        return domain.halfDiagonalTip * domain.halfDiagonalTilt /
               sqrtf(domain.halfDiagonalTip * domain.halfDiagonalTip +
                     domain.halfDiagonalTilt * domain.halfDiagonalTilt);
    }

    /*! Checks that the half-diagonals are in (0, pi/2], so that the rhombus stays within the +-90 degree gimbal limits,
     and that thetaMax is greater than zero and not greater than the inscribed radius of the rhombus. NaN fails every
     comparison.
     @return bool
     @param domain [rad] Addressable gimbal rhombus
    */
    static bool isValidDomain(const GimbalRhombusDomain& domain) {
        constexpr float maxGimbalAngle = std::numbers::pi_v<float> / 2.0F;  // [rad]
        const auto isValidHalfDiagonal = [](float value) { return value > 0.0F && value <= maxGimbalAngle; };
        if (!isValidHalfDiagonal(domain.halfDiagonalTip) || !isValidHalfDiagonal(domain.halfDiagonalTilt)) {
            return false;
        }
        return domain.thetaMax > 0.0F && domain.thetaMax <= inscribedRadius(domain);
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

    uint32_t getStepsPerRevolution() const { return this->stepsPerRevolution; }
    const GimbalRhombusDomain& getDomain() const { return this->domain; }
    const SplineCoefficients& getMotor1Coefficients() const { return this->motor1Coefficients; }
    const SplineCoefficients& getMotor2Coefficients() const { return this->motor2Coefficients; }

   private:
    GimbalAnglesToMotorAnglesConfig(uint32_t stepsPerRevolution,
                                    const GimbalRhombusDomain& domain,
                                    const SplineCoefficients& motor1Coefficients,
                                    const SplineCoefficients& motor2Coefficients)
        : stepsPerRevolution(stepsPerRevolution),
          domain(domain),
          motor1Coefficients(motor1Coefficients),
          motor2Coefficients(motor2Coefficients) {}

    uint32_t stepsPerRevolution;            //!< [steps] Number of motor steps per 2*pi rad
    GimbalRhombusDomain domain;             //!< [rad] Addressable gimbal rhombus
    SplineCoefficients motor1Coefficients;  //!< [steps] Motor 1 step surface control coefficients
    SplineCoefficients motor2Coefficients;  //!< [steps] Motor 2 step surface control coefficients
};

/*! @brief Pure algorithm: determines the stepper motor angles corresponding to the incoming gimbal tip and tilt
 * angles by evaluating one uniform bicubic B-spline surface for each motor. Motor 1 is S1(u, v) and motor 2 is
 * S2(u, v). */
class GimbalAnglesToMotorAnglesAlgorithm final {
   public:
    explicit GimbalAnglesToMotorAnglesAlgorithm(const GimbalAnglesToMotorAnglesConfig& config);
    void setConfig(const GimbalAnglesToMotorAnglesConfig& config);
    void reInitialize();
    GimbalAnglesToMotorAnglesOutput update(float gimbalAngle1, float gimbalAngle2);

   private:
    GimbalAnglesToMotorAnglesOutput motorAngles(float u, float v) const;

    GimbalAnglesToMotorAnglesConfig cfg;
    // Config-derived
    float stepAngle{};                                      //!< [rad/step] Motor step angle, 2*pi / stepsPerRevolution
    GimbalAnglesToMotorAnglesOutput zeroGimbalMotorAngles;  //!< [rad] Motor angles at zero gimbal angles
    // Internal states
    std::optional<GimbalAnglesToMotorAnglesOutput> previousValidOutput;  //!< [rad] Last valid algorithm output
};

#endif /* F32XMERA_GIMBAL_ANGLES_TO_MOTOR_ANGLES_ALGORITHM_H */
