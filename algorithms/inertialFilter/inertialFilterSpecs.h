#ifndef F32XMERA_INERTIALFILTERSPECS_H
#define F32XMERA_INERTIALFILTERSPECS_H

#include "inertialFilterTypes.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"

#include <filteringCore/state.hpp>

#include <Eigen/Core>

#include <variant>

namespace filtering::inertialFilter {

inline constexpr int BatchSize = 2;

// State: [sigma_BN (3), omega_BN_B (3)].
using InertialState = filtering::StateVector<filtering::MrpAttitude<3>, filtering::AngularRate<3>>;

static_assert(InertialState::size == INERTIAL_FILTER_NUM_STATES,
              "INERTIAL_FILTER_NUM_STATES must match the InertialState dimension");

// N x N filter matrix (process noise, covariance) where N = InertialState::size.
using StateMatrix = Eigen::Matrix<float, InertialState::size, InertialState::size>;
// N-element filter column (state seed) where N = InertialState::size.
using StateColumn = Eigen::Vector<float, InertialState::size>;

// MRP kinematics: sigma_dot = 1/4 B(sigma) omega; body rate constant under propagation.
struct InertialDynamics {
    InertialState operator()(float /*t*/, InertialState const& state) const {
        Eigen::Vector3f const sigma = state.get<filtering::MrpAttitude<3>>();
        Eigen::Vector3f const omega = state.get<filtering::AngularRate<3>>();

        InertialState xDot;
        xDot.set<filtering::MrpAttitude<3>>(dmrp(sigma, omega));
        xDot.set<filtering::AngularRate<3>>(Eigen::Vector3f::Zero());
        return xDot;
    }
};

struct StAttMeasurement {
    double timeTag = 0;
    Eigen::Vector3f sigma_BN = Eigen::Vector3f::Zero();
    Eigen::Matrix3f covar = Eigen::Matrix3f::Identity();
    bool valid = false;
};

struct RateMeasurement {
    double timeTag = 0;
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Matrix3f covar = Eigen::Matrix3f::Identity();
    bool valid = false;
};

using Measurement = std::variant<StAttMeasurement, RateMeasurement>;

struct FilterStateOutput {
    static constexpr int N = InertialState::size;
    Eigen::Matrix<float, N, 1> state = Eigen::Matrix<float, N, 1>::Zero();
    Eigen::Matrix<float, N, N> covariance = Eigen::Matrix<float, N, N>::Zero();
};

struct StAttResidualsOutput {
    bool valid = false;
    Eigen::Vector3f observation = Eigen::Vector3f::Zero();
    Eigen::Vector3f preFit = Eigen::Vector3f::Zero();
    Eigen::Vector3f postFit = Eigen::Vector3f::Zero();
};

struct RateResidualsOutput {
    bool valid = false;
    Eigen::Vector3f observation = Eigen::Vector3f::Zero();
    Eigen::Vector3f preFit = Eigen::Vector3f::Zero();
    Eigen::Vector3f postFit = Eigen::Vector3f::Zero();
};

using Residuals = std::variant<StAttResidualsOutput, RateResidualsOutput>;

}  // namespace filtering::inertialFilter

#endif
