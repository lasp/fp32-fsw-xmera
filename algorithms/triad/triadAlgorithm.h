#ifndef F32XMERA_TRIAD_ALGORITHM_H
#define F32XMERA_TRIAD_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <math.h>
#include <Eigen/Core>
#include <numbers>

static constexpr float kParallelThresholdRad = 0.5F * std::numbers::pi_v<float> / 180.0F;

class TriadConfig final {
   public:
    static TriadConfig create(const Eigen::Vector3f& a1Hat_B,
                              const Eigen::Vector3f& hHat_N,
                              const float signOfZHat_N) {
        if (!isValidA1Hat_B(a1Hat_B)) {
            FSW_THROW_INVALID_ARGUMENT("triad: a1Hat_B must be a unit vector");
        }
        if (!isValidHHat_N(hHat_N)) {
            FSW_THROW_INVALID_ARGUMENT("triad: hHat_N must be a unit vector");
        }
        if (!isValidSignOfZHat_N(signOfZHat_N)) {
            FSW_THROW_INVALID_ARGUMENT("triad: signOfZHat_N cannot be zero");
        }
        return {a1Hat_B.normalized(), hHat_N.normalized(), copysignf(1.0F, signOfZHat_N)};
    }

    static bool isValidA1Hat_B(const Eigen::Vector3f& a1Hat_B) { return fabsf(a1Hat_B.stableNorm() - 1.0F) < 1e-3F; }
    static bool isValidHHat_N(const Eigen::Vector3f& hHat_N) { return fabsf(hHat_N.stableNorm() - 1.0F) < 1e-3F; }
    static bool isValidSignOfZHat_N(const float signOfZHat_N) { return signOfZHat_N != 0.0F; }

    Eigen::Vector3f getA1Hat_B() const { return a1Hat_B; }
    Eigen::Vector3f getHHat_N() const { return hHat_N; }
    float getSignOfZHat_N() const { return signOfZHat_N; }

   private:
    TriadConfig(const Eigen::Vector3f& a1Hat_B,
                const Eigen::Vector3f& hHat_N,
                const float signOfZHat_N)
        : a1Hat_B(a1Hat_B), hHat_N(hHat_N), signOfZHat_N(signOfZHat_N) {}

    Eigen::Vector3f a1Hat_B{Eigen::Vector3f::Zero()};
    Eigen::Vector3f hHat_N{Eigen::Vector3f::Zero()};
    float signOfZHat_N{};
};

class TriadAlgorithm final {
   public:
    explicit TriadAlgorithm(const TriadConfig& config);

    void setConfig(const TriadConfig& config);

    Eigen::Vector3f update(const Eigen::Vector3f& sigma_BN,
                           const Eigen::Vector3f& rHat_SB_B,
                           const Eigen::Vector3f& hRefHat_B) const;

   private:
    TriadConfig cfg;
};

#endif
