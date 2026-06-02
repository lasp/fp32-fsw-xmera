#ifndef F32XMERA_TRIAD_ALGORITHM_H
#define F32XMERA_TRIAD_ALGORITHM_H

#include "utilities/fsw/freestandingInvalidArgument.h"
#include <Eigen/Core>

class TriadConfig final {
   public:
    static TriadConfig create(const Eigen::Vector3f& a1Hat_B, const Eigen::Vector3f& hHat_N) {
        if (!isValidA1Hat_B(a1Hat_B)) {
            FSW_THROW_INVALID_ARGUMENT("triad: a1Hat_B must be a non-zero vector");
        }
        if (!isValidHHat_N(hHat_N)) {
            FSW_THROW_INVALID_ARGUMENT("triad: hHat_N must be a non-zero vector");
        }
        return {a1Hat_B, hHat_N};
    }

    static bool isValidA1Hat_B(const Eigen::Vector3f& a1Hat_B) { return a1Hat_B.norm() > 1e-6F; }
    static bool isValidHHat_N(const Eigen::Vector3f& hHat_N) { return hHat_N.norm() > 1e-6F; }

    Eigen::Vector3f getA1Hat_B() const { return a1Hat_B; }
    Eigen::Vector3f getHHat_N() const { return hHat_N; }

   private:
    TriadConfig(const Eigen::Vector3f& a1Hat_B, const Eigen::Vector3f& hHat_N) : a1Hat_B(a1Hat_B), hHat_N(hHat_N) {}

    Eigen::Vector3f a1Hat_B{Eigen::Vector3f::Zero()};
    Eigen::Vector3f hHat_N{Eigen::Vector3f::Zero()};
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
