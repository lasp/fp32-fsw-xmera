#ifndef F32XMERA_TRIAD_ALGORITHM_H
#define F32XMERA_TRIAD_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <math.h>
#include <Eigen/Core>
#include <numbers>

static constexpr float kParallelThresholdRad = 0.5F * std::numbers::pi_v<float> / 180.0F;

enum class N3Axis { plusZHat_N = 0, minusZHat_N = 1 };

class TriadConfig final {
   public:
    static TriadConfig create(const Eigen::Vector3f& sadaHat_B,
                              const Eigen::Vector3f& thrustReqHat_N,
                              const N3Axis n3Axis) {
        if (!isValidSadaHat_B(sadaHat_B)) {
            FSW_THROW_INVALID_ARGUMENT("triad: sadaHat_B must be a unit vector");
        }
        if (!isValidThrustReqHat_N(thrustReqHat_N)) {
            FSW_THROW_INVALID_ARGUMENT("triad: thrustReqHat_N must be a unit vector");
        }
        if (!isValidN3Axis(n3Axis)) {
            FSW_THROW_INVALID_ARGUMENT("triad: n3Axis must be plusZHat_N or minusZHat_N");
        }
        return {sadaHat_B.normalized(), thrustReqHat_N.normalized(), n3Axis};
    }

    static bool isValidSadaHat_B(const Eigen::Vector3f& sadaHat_B) {
        return fabsf(sadaHat_B.stableNorm() - 1.0F) < 1e-3F;
    }
    static bool isValidThrustReqHat_N(const Eigen::Vector3f& thrustReqHat_N) {
        return fabsf(thrustReqHat_N.stableNorm() - 1.0F) < 1e-3F;
    }
    static bool isValidN3Axis(const N3Axis n3Axis) {
        return n3Axis == N3Axis::plusZHat_N || n3Axis == N3Axis::minusZHat_N;
    }

    Eigen::Vector3f getSadaHat_B() const { return sadaHat_B; }
    Eigen::Vector3f getThrustReqHat_N() const { return thrustReqHat_N; }
    N3Axis getN3Axis() const { return n3Axis; }

   private:
    TriadConfig(const Eigen::Vector3f& sadaHat_B, const Eigen::Vector3f& thrustReqHat_N, const N3Axis n3Axis)
        : sadaHat_B(sadaHat_B), thrustReqHat_N(thrustReqHat_N), n3Axis(n3Axis) {}

    Eigen::Vector3f sadaHat_B{Eigen::Vector3f::Zero()};
    Eigen::Vector3f thrustReqHat_N{Eigen::Vector3f::Zero()};
    N3Axis n3Axis{N3Axis::plusZHat_N};
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
