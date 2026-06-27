#ifndef F32XMERA_INERTIALSRUKFSPECS_H
#define F32XMERA_INERTIALSRUKFSPECS_H

#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <filteringCore/state.hpp>

#include <Eigen/Core>

#include <variant>

namespace filtering::inertialSRuKF {

inline constexpr int BatchSize = 2;

// State: [sigma_BN (3), omega_BN_B (3)].
using InertialState = filtering::StateVector<filtering::MrpAttitude<3>, filtering::AngularRate<3>>;

// N x N filter matrix (process noise, covariance) where N = InertialState::size.
using StateMatrix = Eigen::Matrix<double, InertialState::size, InertialState::size>;

// MRP kinematics: sigma_dot = 1/4 B(sigma) omega; body rate constant under propagation.
struct InertialDynamics {
    InertialState operator()(double /*t*/, InertialState const& state) const {
        Eigen::Vector3d const sigma = state.get<filtering::MrpAttitude<3>>();
        Eigen::Vector3d const omega = state.get<filtering::AngularRate<3>>();

        InertialState xDot;
        xDot.set<filtering::MrpAttitude<3>>(0.25 * bmatMrp(sigma) * omega);
        xDot.set<filtering::AngularRate<3>>(Eigen::Vector3d::Zero());
        return xDot;
    }
};

struct StAttMeasurement {
    double timeTag = 0;
    Eigen::Vector3d sigma_BN = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covar = Eigen::Matrix3d::Identity();
    bool valid = false;
};

struct RateMeasurement {
    double timeTag = 0;
    Eigen::Vector3d omega_BN_B = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covar = Eigen::Matrix3d::Identity();
    bool valid = false;
};

using Measurement = std::variant<StAttMeasurement, RateMeasurement>;

struct FilterStateOutput {
    static constexpr int N = InertialState::size;
    Eigen::Matrix<double, N, 1> state = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, N> covariance = Eigen::Matrix<double, N, N>::Zero();
};

struct StAttResidualsOutput {
    bool valid = false;
    Eigen::Vector3d observation = Eigen::Vector3d::Zero();
    Eigen::Vector3d preFit = Eigen::Vector3d::Zero();
    Eigen::Vector3d postFit = Eigen::Vector3d::Zero();
};

struct RateResidualsOutput {
    bool valid = false;
    Eigen::Vector3d observation = Eigen::Vector3d::Zero();
    Eigen::Vector3d preFit = Eigen::Vector3d::Zero();
    Eigen::Vector3d postFit = Eigen::Vector3d::Zero();
};

using Residuals = std::variant<StAttResidualsOutput, RateResidualsOutput>;

}  // namespace filtering::inertialSRuKF

#endif
