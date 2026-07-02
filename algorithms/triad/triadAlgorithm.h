#ifndef F32XMERA_TRIAD_ALGORITHM_H
#define F32XMERA_TRIAD_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <math.h>
#include <Eigen/Core>
#include <numbers>

static constexpr float kParallelThresholdRad = 0.5F * std::numbers::pi_v<float> / 180.0F;

class TriadConfig final {
   public:
    static TriadConfig create(const Eigen::Vector3f& sadaHat_B,
                              const Eigen::Vector3f& thrustReqHat_N,
                              const float signOfN3Hat_N) {
        if (!isValidSadaHat_B(sadaHat_B)) {
            FSW_THROW_INVALID_ARGUMENT("triad: sadaHat_B must be a unit vector");
        }
        if (!isValidThrustReqHat_N(thrustReqHat_N)) {
            FSW_THROW_INVALID_ARGUMENT("triad: thrustReqHat_N must be a unit vector");
        }
        if (!isValidSignOfN3Hat_N(signOfN3Hat_N)) {
            FSW_THROW_INVALID_ARGUMENT("triad: signOfN3Hat_N cannot be zero");
        }
        return {sadaHat_B.normalized(), thrustReqHat_N.normalized(), copysignf(1.0F, signOfN3Hat_N)};
    }

    static bool isValidSadaHat_B(const Eigen::Vector3f& sadaHat_B) {
        return fabsf(sadaHat_B.stableNorm() - 1.0F) < 1e-3F;
    }
    static bool isValidThrustReqHat_N(const Eigen::Vector3f& thrustReqHat_N) {
        return fabsf(thrustReqHat_N.stableNorm() - 1.0F) < 1e-3F;
    }
    static bool isValidSignOfN3Hat_N(const float signOfN3Hat_N) { return signOfN3Hat_N != 0.0F; }

    Eigen::Vector3f getSadaHat_B() const { return sadaHat_B; }
    Eigen::Vector3f getThrustReqHat_N() const { return thrustReqHat_N; }
    float getSignOfN3Hat_N() const { return signOfN3Hat_N; }

   private:
    TriadConfig(const Eigen::Vector3f& sadaHat_B, const Eigen::Vector3f& thrustReqHat_N, const float signOfN3Hat_N)
        : sadaHat_B(sadaHat_B), thrustReqHat_N(thrustReqHat_N), signOfN3Hat_N(signOfN3Hat_N) {}

    Eigen::Vector3f sadaHat_B{Eigen::Vector3f::Zero()};
    Eigen::Vector3f thrustReqHat_N{Eigen::Vector3f::Zero()};
    float signOfN3Hat_N{};
};

class TriadAlgorithm final {
   public:
    explicit TriadAlgorithm(const TriadConfig& config);

    void setConfig(const TriadConfig& config);

    Eigen::Vector3f update(const Eigen::Vector3f& rHat_SB_N, const Eigen::Vector3f& thrustHat_B) const;

   private:
    TriadConfig cfg;
};

#endif
