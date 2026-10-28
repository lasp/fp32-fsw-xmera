#ifndef F32XMERA_FLYBYFILTERSPECS_H
#define F32XMERA_FLYBYFILTERSPECS_H

#include <filteringCore/state.hpp>

#include <Eigen/Core>

namespace filtering::flybyFilter {

inline constexpr int BatchSize = 2;

// State: [r_BN_N (3), v_BN_N (3), b (2)]. The position and velocity are in the inertial frame N, in internal
// units of km and km/s (the adapter converts them to/from SI at the message boundary). b is a constant heading
// bias of two angles [rad] about the line of sight: across the terminator (toward the sun, perpendicular to the
// line of sight) and along it.
using FlybyState = filtering::StateVector<filtering::Position<3>, filtering::Velocity<3>, filtering::Bias<2>>;

// Indices of the two bias angles in the Bias<2> component.
inline constexpr int AcrossTerminator = 0;
inline constexpr int AlongTerminator = 1;

// Smallest angle between the sun direction and the measured line of sight for which the heading bias is modeled
// (MinSunLineOfSightAngle), and its sine: the smallest norm of the sun direction's part perpendicular to the line of
// sight (MinSunProjection). Closer to 0 or 180 deg phase, or with no sun direction, the terminator directions are
// ill-defined and a heading is modeled without bias.
inline constexpr float MinSunLineOfSightAngle = 5.0F * 3.14159265F / 180.0F;  // [rad]
inline constexpr float MinSunProjection = 0.0871557427F;                      // [-] sin(MinSunLineOfSightAngle)

// N x N filter matrix (process noise, covariance) where N = FlybyState::size.
using StateMatrix = Eigen::Matrix<float, FlybyState::size, FlybyState::size>;
// N-element filter column (state seed) where N = FlybyState::size.
using StateColumn = Eigen::Vector<float, FlybyState::size>;

inline constexpr float MinGravityRange = 1e-3F;
// Two-body point-mass gravity: r_dot = v; v_dot = -mu/max(|r|, MinGravityRange)^3 r. The
// gravitational parameter mu is carried by the functor (internal units km^3/s^2) and set from the
// configuration. The denominator (range) is clamped to keep the acceleration
// continuous and bounded. The heading bias is constant: b_dot = 0.
struct FlybyDynamics {
    float mu = 0.0F;

    FlybyState operator()(float /*t*/, FlybyState const& state) const {
        Eigen::Vector3f const r = state.get<filtering::Position<3>>();
        Eigen::Vector3f const v = state.get<filtering::Velocity<3>>();

        float const norm = r.stableNorm();
        float const range = norm < MinGravityRange ? MinGravityRange : norm;
        float const rangeCubed = range * range * range;

        FlybyState xDot;
        xDot.set<filtering::Position<3>>(v);
        xDot.set<filtering::Velocity<3>>(-this->mu / rangeCubed * r);
        xDot.set<filtering::Bias<2>>(Eigen::Vector2f::Zero());
        return xDot;
    }
};

// Optical-navigation heading measurement: a unit vector rhat_BN_N (spacecraft-to-target, inertial
// frame). covar is the measurement noise (dimensionless, the unit vector is frame-only). shat_N is the
// unit sun direction (spacecraft-to-sun, inertial frame) at the reading's time, or zero if unavailable.
struct HeadingMeasurement {
    double timeTag = 0;
    Eigen::Vector3f rhat_BN_N = Eigen::Vector3f::Zero();
    Eigen::Vector3f shat_N = Eigen::Vector3f::Zero();
    Eigen::Matrix3f covar = Eigen::Matrix3f::Identity();
    bool valid = false;
};

// A single measurement kind (target heading).
using Measurement = HeadingMeasurement;

struct FilterStateOutput {
    static constexpr int N = FlybyState::size;
    Eigen::Matrix<float, N, 1> state = Eigen::Matrix<float, N, 1>::Zero();
    Eigen::Matrix<float, N, N> covariance = Eigen::Matrix<float, N, N>::Zero();
};

struct HeadingResidualsOutput {
    bool valid = false;
    Eigen::Vector3f observation = Eigen::Vector3f::Zero();
    Eigen::Vector3f preFit = Eigen::Vector3f::Zero();
    Eigen::Vector3f postFit = Eigen::Vector3f::Zero();
};

}  // namespace filtering::flybyFilter

#endif
