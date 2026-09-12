#ifndef F32XMERA_THRUST_VECTORING_ALGORITHM_H
#define F32XMERA_THRUST_VECTORING_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include "utilities/fsw/freestandingIsFinite.hpp"

#include <Eigen/Core>

//! [m] smallest center-of-mass offset from the thrust point M for which a thrust direction is defined
inline constexpr float kMinR_CM = 1e-3F;

/*!
 * @brief Validated configuration for the thrust vectoring algorithm.
 *
 * Bundles the thrust point, the thrust magnitude and the center-of-mass position, all of which are fixed while
 * the module runs. An instance can only exist if the positions are finite, the thrust magnitude is finite and
 * positive, and the center of mass is far enough from the thrust point for a direction to be defined.
 * Construct via ThrustVectoringConfig::create(...).
 */
class ThrustVectoringConfig final {
   public:
    static ThrustVectoringConfig create(const Eigen::Vector3f& r_MB_B, float thrust, const Eigen::Vector3f& r_CB_B) {
        if (!isValidR_MB_B(r_MB_B)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_MB_B must be finite.");
        }
        if (!isValidThrust(thrust)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: thrust must be finite and positive.");
        }
        if (!isValidR_CB_B(r_CB_B)) {
            FSW_THROW_INVALID_ARGUMENT("thrustVectoring: r_CB_B must be finite.");
        }
        if (!isValidR_CM(r_CB_B, r_MB_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrustVectoring: the center of mass must be farther than kMinR_CM from the thrust point M, "
                "otherwise no thrust direction is defined.");
        }
        if (!isValidMaxAchievableTorque(thrust, r_CB_B, r_MB_B)) {
            FSW_THROW_INVALID_ARGUMENT(
                "thrustVectoring: the thrust times the moment arm must be finite and positive, otherwise the "
                "largest torque the geometry can deliver is not representable.");
        }

        return {r_MB_B, thrust, r_CB_B};
    }

    static bool isValidR_MB_B(const Eigen::Vector3f& r_MB_B) { return r_MB_B.allFinite(); }
    /*! A zero thrust produces no torque about any point, so no direction solves the request. */
    static bool isValidThrust(float thrust) { return fsw::is_finite(thrust) && thrust > 0.0F; }
    static bool isValidR_CB_B(const Eigen::Vector3f& r_CB_B) { return r_CB_B.allFinite(); }
    /*! Both endpoints can be finite and still lie far enough apart that their difference is not representable,
     * which would leave the moment arm infinite and its direction undefined. */
    static bool isValidR_CM(const Eigen::Vector3f& r_CB_B, const Eigen::Vector3f& r_MB_B) {
        const Eigen::Vector3f r_MC_B = r_MB_B - r_CB_B;
        return r_MC_B.allFinite() && r_MC_B.stableNorm() > kMinR_CM;
    }
    /*! The solve divides by the largest torque the geometry can deliver. The thrust and the moment arm can each
     * pass their own check and still multiply to zero or to infinity in single precision. Call only for a
     * geometry isValidR_CM has already accepted. */
    static bool isValidMaxAchievableTorque(float thrust, const Eigen::Vector3f& r_CB_B, const Eigen::Vector3f& r_MB_B) {
        const float maxAchievableTorque = thrust * (r_MB_B - r_CB_B).stableNorm();
        return fsw::is_finite(maxAchievableTorque) && maxAchievableTorque > 0.0F;
    }

    const Eigen::Vector3f& getR_MB_B() const { return this->r_MB_B; }
    float getThrust() const { return this->thrust; }
    const Eigen::Vector3f& getR_CB_B() const { return this->r_CB_B; }

   private:
    // NOLINTBEGIN(modernize-pass-by-value)
    // modernize-pass-by-value: this is a private constructor invoked only from create() with already-validated
    //   arguments; the small vectors are stored by copy without a move for clarity.
    ThrustVectoringConfig(const Eigen::Vector3f& r_MB_B, float thrust, const Eigen::Vector3f& r_CB_B)
        : r_MB_B(r_MB_B), thrust(thrust), r_CB_B(r_CB_B) {}
    // NOLINTEND(modernize-pass-by-value)

    Eigen::Vector3f r_MB_B;  //!< [m] thrust point M w.r.t. B origin, B frame
    float thrust;            //!< [N] thrust magnitude
    Eigen::Vector3f r_CB_B;  //!< [m] center of mass w.r.t. B origin, B frame
};

/*! @brief Pure algorithm computing the direction of a thrust acting through a single point.
 *
 * The thrust has a fixed magnitude and its line of action passes through the fixed point M, so the only freedom
 * left is the direction. The torque it produces about the center of mass therefore depends on nothing but that
 * direction, which makes the solve closed form, with no state and no dependence on the previous cycle. The
 * algorithm holds no description of the mechanism that aims the thrust.
 */
class ThrustVectoringAlgorithm final {
   public:
    explicit ThrustVectoringAlgorithm(const ThrustVectoringConfig& config);
    void setConfig(const ThrustVectoringConfig& config);
    const ThrustVectoringConfig& getConfig() const { return this->cfg; }
    /*! @return [-] thrust unit direction, B frame */
    Eigen::Vector3f update(const Eigen::Vector3f& Lreq_B) const;

   private:
    ThrustVectoringConfig cfg;  //!< [-] validated configuration
    Eigen::Vector3f r_MC_B{Eigen::Vector3f::UnitZ()};
};

#endif  // F32XMERA_THRUST_VECTORING_ALGORITHM_H
