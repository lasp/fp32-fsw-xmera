#ifndef F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_ALGORITHM_H
#define F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_ALGORITHM_H

#include <math.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <numbers>

#include "gimbalToMotorAnglesSplineTypes.h"
#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

static constexpr float kSplineDefaultMotorAngle = 103.2242F * std::numbers::pi_v<float> / 180.0F;
static constexpr int kNumSplineIntervals = NUM_SPLINE_INTERVALS;
static constexpr int kNumSplineCoeffsPerAxis = NUM_SPLINE_COEFFS_PER_AXIS;

/*! @brief Structure containing the algorithm output. */
struct GimbalToMotorAnglesSplineOutput {
    float motorAngle1{kSplineDefaultMotorAngle};  //!< [rad] Motor 1 angle
    float motorAngle2{kSplineDefaultMotorAngle};  //!< [rad] Motor 2 angle
};

/*! @brief Addressable gimbal rhombus |u| <= 1, |v| <= 1, where
 * u = (tip - centerTipAngle) / halfDiagonalTip + (tilt - centerTiltAngle) / halfDiagonalTilt and
 * v = (tip - centerTipAngle) / halfDiagonalTip - (tilt - centerTiltAngle) / halfDiagonalTilt. */
struct GimbalRhombusDomain {
    float centerTipAngle;    //!< [rad] Gimbal tip angle at the rhombus center
    float centerTiltAngle;   //!< [rad] Gimbal tilt angle at the rhombus center (motor mirror axis)
    float halfDiagonalTip;   //!< [rad] Half of the rhombus diagonal along the gimbal tip axis
    float halfDiagonalTilt;  //!< [rad] Half of the rhombus diagonal along the gimbal tilt axis
};

/*! @brief Type alias for the uniform bicubic B-spline control coefficients [rad]. Coefficient c[i][j] is stored at
 * index i * NUM_SPLINE_COEFFS_PER_AXIS + j, where i is along u and j is along v. */
using SplineCoefficients = std::array<float, static_cast<std::size_t>(NUM_SPLINE_COEFFS)>;

/*! @brief Validated configuration for the gimbal to motor angles spline algorithm.
 * Holds the addressable gimbal rhombus and the control coefficients of the shared motor angle surface. */
class GimbalToMotorAnglesSplineConfig final {
   public:
    static GimbalToMotorAnglesSplineConfig create(const GimbalRhombusDomain& domain,
                                                  const SplineCoefficients& coefficients) {
        if (!isValidDomain(domain)) {
            FSW_THROW_INVALID_ARGUMENT(
                "gimbalToMotorAnglesSpline: the rhombus values must be finite, the half-diagonals must be greater "
                "than zero, and the rhombus must stay within +-pi/2 on each gimbal axis.");
        }
        if (!isValidCoefficients(coefficients)) {
            FSW_THROW_INVALID_ARGUMENT("gimbalToMotorAnglesSpline: all spline coefficients must be finite.");
        }

        return {domain, coefficients};
    }

    /*! Checks that the rhombus is finite, not degenerate, and within the +-90 degree gimbal limits.
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

    /*! Checks that every spline coefficient is finite.
     @return bool
     @param coefficients [rad] Spline control coefficients
    */
    static bool isValidCoefficients(const SplineCoefficients& coefficients) {
        return std::ranges::all_of(coefficients, [](float value) { return fsw::is_finite(value); });
    }

    const GimbalRhombusDomain& getDomain() const { return this->domain; }
    const SplineCoefficients& getCoefficients() const { return this->coefficients; }

   private:
    GimbalToMotorAnglesSplineConfig(const GimbalRhombusDomain& domain, const SplineCoefficients& coefficients)
        : domain(domain), coefficients(coefficients) {}

    GimbalRhombusDomain domain;       //!< [rad] Addressable gimbal rhombus
    SplineCoefficients coefficients;  //!< [rad] Shared motor angle surface control coefficients
};

/*! @brief Pure algorithm: determines the stepper motor angles corresponding to the incoming gimbal tip and tilt
 * angles by evaluating one shared uniform bicubic B-spline surface S. Motor 1 is S(u, v) and motor 2 is S(v, u). */
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
    float evaluateSurface(float u, float v) const;
    GimbalToMotorAnglesSplineOutput previousValidOutput{};  //!< [rad] Last valid algorithm output
    GimbalToMotorAnglesSplineConfig cfg;
};

#endif /* F32XMERA_GIMBAL_TO_MOTOR_ANGLES_SPLINE_ALGORITHM_H */
