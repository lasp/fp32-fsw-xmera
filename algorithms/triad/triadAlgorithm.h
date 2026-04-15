#ifndef F32XMERA_TRIAD_ALGORITHM_H
#define F32XMERA_TRIAD_ALGORITHM_H

#include "freestandingInvalidArgument.h"

#include <Eigen/Core>

#include <stdint.h>

enum class CelestialBody : uint8_t { NotSun = 0, Sun = 1 };

class TriadConfig final {
   public:
    static TriadConfig create(const Eigen::Vector3f& a1Hat_B,
                              const Eigen::Vector3f& h1Hat_B,
                              const Eigen::Vector3f& hHat_N,
                              const CelestialBody celestialBodyInput) {
        if (!isValidA1Hat_B(a1Hat_B)) {
            FS_THROW_INVALID_ARGUMENT("triad: a1Hat_B must be a non-zero vector");
        }
        return {a1Hat_B, h1Hat_B, hHat_N, celestialBodyInput};
    }

    static bool isValidA1Hat_B(const Eigen::Vector3f& a1Hat_B) { return a1Hat_B.norm() > 1e-6F; }
    static bool isValidH1Hat_B(const Eigen::Vector3f& /*h1Hat_B*/) { return true; }
    static bool isValidHHat_N(const Eigen::Vector3f& /*hHat_N*/) { return true; }
    static bool isValidCelestialBodyInput(const CelestialBody /*celestialBodyInput*/) { return true; }

    Eigen::Vector3f getA1Hat_B() const { return a1Hat_B; }
    Eigen::Vector3f getH1Hat_B() const { return h1Hat_B; }
    Eigen::Vector3f getHHat_N() const { return hHat_N; }
    CelestialBody getCelestialBodyInput() const { return celestialBodyInput; }

   private:
    TriadConfig(const Eigen::Vector3f& a1Hat_B,
                const Eigen::Vector3f& h1Hat_B,
                const Eigen::Vector3f& hHat_N,
                const CelestialBody celestialBodyInput)
        : a1Hat_B(a1Hat_B), h1Hat_B(h1Hat_B), hHat_N(hHat_N), celestialBodyInput(celestialBodyInput) {}

    Eigen::Vector3f a1Hat_B;
    Eigen::Vector3f h1Hat_B;
    Eigen::Vector3f hHat_N;
    CelestialBody celestialBodyInput;
};

class TriadAlgorithm final {
   public:
    Eigen::Vector3f update(const Eigen::Vector3f& rHat_SB_N,
                           const Eigen::Vector3f& hReqHat_N,
                           const Eigen::Vector3f& hRefHat_B) const;

    void setA1Hat_B(const Eigen::Vector3f& a1Hat_B);
    Eigen::Vector3f getA1Hat_B() const;

   private:
    Eigen::Vector3f a1Hat_B = Eigen::Vector3f::Zero();
};

#endif
