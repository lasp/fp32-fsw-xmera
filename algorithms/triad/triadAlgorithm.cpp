#include "triadAlgorithm.h"

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include <math.h>
#include <Eigen/Core>
#include <numbers>
#include <stdexcept>

static constexpr float kSpeParallelThresholdDeg = 0.5F;
static constexpr float kRadToDeg = 180.0F / std::numbers::pi_v<float>;

static float SPE_angle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    const float dot = v1.dot(v2);
    const float cross = v1.x() * v2.y() - v1.y() * v2.x();

    float angle = safeAcosf(dot / (v1.norm() * v2.norm()));
    angle = angle * kRadToDeg;

    if (cross < 0.0F) {
        angle = -angle;
    }

    return angle;
}

TriadAlgorithm::TriadAlgorithm(const TriadConfig& config) : cfg(config) {}

void TriadAlgorithm::setConfig(const TriadConfig& config) { this->cfg = config; }

Eigen::Vector3f TriadAlgorithm::update(const Eigen::Vector3f& sigma_BN,
                                       const Eigen::Vector3f& rHat_SB_B,
                                       const Eigen::Vector3f& hReqHat_N,
                                       const Eigen::Vector3f& hRefHat_B) const {
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Vector3f rHat_SB_N = (dcm_BN.transpose() * rHat_SB_B).normalized();
    if (const float SPE = SPE_angle(rHat_SB_N, hReqHat_N); fabsf(SPE) < kSpeParallelThresholdDeg) {
        throw std::runtime_error("sun and earth reference vectors are parallel, Triad can not be used");
    }

    const Eigen::Vector3f a1 = this->cfg.getA1Hat_B().normalized();

    Eigen::Matrix3f RD;
    const Eigen::Vector3f r2 = hRefHat_B;
    const Eigen::Vector3f r3 = a1.cross(hRefHat_B).normalized();
    const Eigen::Vector3f r1 = r2.cross(r3);
    RD.col(0) = r1;
    RD.col(1) = r2;
    RD.col(2) = r3;

    Eigen::Matrix3f BD;
    const Eigen::Vector3f n2 = hReqHat_N;
    const Eigen::Vector3f n1 = rHat_SB_N.cross(hReqHat_N).normalized();
    const Eigen::Vector3f n3 = n1.cross(n2);
    BD.col(0) = n1;
    BD.col(1) = n2;
    BD.col(2) = n3;

    const Eigen::Matrix3f RN = RD * BD.transpose();

    return dcmToMrp(RN);
}
