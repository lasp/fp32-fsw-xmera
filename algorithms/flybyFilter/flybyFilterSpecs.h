#ifndef F32XMERA_FLYBYFILTERSPECS_H
#define F32XMERA_FLYBYFILTERSPECS_H

#include <filteringCore/state.hpp>

#include <Eigen/Core>

namespace filtering::flybyFilter {

inline constexpr int BatchSize = 1;

// State: [r_BN_N (3), v_BN_N (3)] in the inertial frame N. Internal units are km and km/s (the
// adapter converts to/from SI at the message boundary).
using FlybyState = filtering::StateVector<filtering::Position<3>, filtering::Velocity<3>>;

// N x N filter matrix (process noise, covariance) where N = FlybyState::size.
using StateMatrix = Eigen::Matrix<double, FlybyState::size, FlybyState::size>;

// Two-body point-mass gravity: r_dot = v; v_dot = -mu/|r|^3 r. The gravitational parameter mu is
// carried by the functor (internal units km^3/s^2) and set from the configuration.
struct FlybyDynamics {
    double mu = 0.0;

    FlybyState operator()(double /*t*/, FlybyState const& state) const {
        Eigen::Vector3d const r = state.get<filtering::Position<3>>();
        Eigen::Vector3d const v = state.get<filtering::Velocity<3>>();

        FlybyState xDot;
        xDot.set<filtering::Position<3>>(v);
        xDot.set<filtering::Velocity<3>>(-this->mu / std::pow(r.norm(), 3) * r);
        return xDot;
    }
};

// Optical-navigation heading measurement: a unit vector rhat_BN_N (spacecraft-to-target, inertial
// frame). covar is the measurement noise (dimensionless, the unit vector is frame-only).
struct HeadingMeasurement {
    double timeTag = 0;
    Eigen::Vector3d rhat_BN_N = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covar = Eigen::Matrix3d::Identity();
    bool valid = false;
};

// A single measurement kind (target heading).
using Measurement = HeadingMeasurement;

struct FilterStateOutput {
    static constexpr int N = FlybyState::size;
    Eigen::Matrix<double, N, 1> state = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, N> covariance = Eigen::Matrix<double, N, N>::Zero();
};

struct HeadingResidualsOutput {
    bool valid = false;
    Eigen::Vector3d observation = Eigen::Vector3d::Zero();
    Eigen::Vector3d preFit = Eigen::Vector3d::Zero();
    Eigen::Vector3d postFit = Eigen::Vector3d::Zero();
};

}  // namespace filtering::flybyFilter

#endif
