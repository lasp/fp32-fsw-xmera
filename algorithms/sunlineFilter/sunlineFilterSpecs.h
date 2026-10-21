#ifndef F32XMERA_SUNLINEFILTERSPECS_H
#define F32XMERA_SUNLINEFILTERSPECS_H

#include "sunlineFilterTypes.h"

#include <filteringCore/state.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <variant>

namespace filtering::sunlineFilter {

inline constexpr int MaxCss = SUNLINE_FILTER_MAX_CSS;
inline constexpr int BatchSize = 2;

// State: [s_hat (3), omega_BN_B (3), bias (1)].
using SunlineState = filtering::StateVector<filtering::Position<3>, filtering::Velocity<3>, filtering::Bias<1>>;

static_assert(SunlineState::size == SUNLINE_FILTER_NUM_STATES,
              "SUNLINE_FILTER_NUM_STATES must match the SunlineState dimension");

// N x N filter matrix (process noise, covariance) where N = SunlineState::size.
using StateMatrix = Eigen::Matrix<float, SunlineState::size, SunlineState::size>;

// ds/dt = s × omega; omega and bias constant under propagation.
struct SunlineDynamics {
    SunlineState operator()(float /*t*/, SunlineState const& state) const {
        Eigen::Vector3f const sHat = state.get<filtering::Position<3>>();
        Eigen::Vector3f const omega = state.get<filtering::Velocity<3>>();

        SunlineState xDot;
        xDot.set<filtering::Position<3>>(sHat.cross(omega));
        xDot.set<filtering::Velocity<3>>(Eigen::Vector3f::Zero());
        xDot.set<filtering::Bias<1>>(Eigen::Vector<float, 1>::Zero());
        return xDot;
    }
};

struct CssMeasurement {
    double timeTag = 0;
    Eigen::Vector<float, MaxCss> cssCosValues = Eigen::Vector<float, MaxCss>::Zero();
    Eigen::Matrix<float, MaxCss, 3> hMatrix = Eigen::Matrix<float, MaxCss, 3>::Zero();
    Eigen::Matrix<float, MaxCss, MaxCss> covar = Eigen::Matrix<float, MaxCss, MaxCss>::Identity();
    int numberOfActiveCss = 0;
    bool valid = false;
};

struct RateMeasurement {
    double timeTag = 0;
    Eigen::Vector3f omega_BN_B = Eigen::Vector3f::Zero();
    Eigen::Matrix3f covar = Eigen::Matrix3f::Identity();
    bool valid = false;
};

using Measurement = std::variant<CssMeasurement, RateMeasurement>;

struct FilterStateOutput {
    static constexpr int N = SunlineState::size;
    Eigen::Matrix<float, N, 1> state = Eigen::Matrix<float, N, 1>::Zero();
    Eigen::Matrix<float, N, N> covariance = Eigen::Matrix<float, N, N>::Zero();
};

struct CssResidualsOutput {
    bool valid = false;
    int numberOfActiveCss = 0;
    Eigen::Vector<float, MaxCss> observation = Eigen::Vector<float, MaxCss>::Zero();
    Eigen::Vector<float, MaxCss> preFit = Eigen::Vector<float, MaxCss>::Zero();
    Eigen::Vector<float, MaxCss> postFit = Eigen::Vector<float, MaxCss>::Zero();
};

struct RateResidualsOutput {
    bool valid = false;
    Eigen::Vector3f observation = Eigen::Vector3f::Zero();
    Eigen::Vector3f preFit = Eigen::Vector3f::Zero();
    Eigen::Vector3f postFit = Eigen::Vector3f::Zero();
};

using Residuals = std::variant<CssResidualsOutput, RateResidualsOutput>;

}  // namespace filtering::sunlineFilter

#endif
