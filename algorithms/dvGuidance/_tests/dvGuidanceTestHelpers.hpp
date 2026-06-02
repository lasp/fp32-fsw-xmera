#ifndef TEST_DV_GUIDANCE_HELPERS_H
#define TEST_DV_GUIDANCE_HELPERS_H

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "dvGuidanceAlgorithm.h"
#include "dvGuidanceTypes.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

struct ReferenceDvGuidanceOutput {
    Eigen::Matrix3d dcm_RN;
    Eigen::Vector3d omega_RN_N;
    Eigen::Vector3d domega_RN_N;
};

// Independent reference implementation kept in pure double precision, used to verify the
// algorithm's FP32 output to within float tolerance. Attitude is returned as a DCM (not an MRP)
// so the test comparison is robust at the MRP shadow-set boundary (|sigma|=1 at 180 deg). The
// degenerate-input and small-angle guards mirror DvGuidanceAlgorithm::update() (reusing the same
// threshold constants) so the reference stays in lockstep with the algorithm.
inline ReferenceDvGuidanceOutput referenceDvGuidance(const Eigen::Vector3d& dvInrtlCmd,
                                                     const Eigen::Vector3d& dvRotVecUnit,
                                                     double dvRotVecMag,
                                                     uint64_t burnStartTime,
                                                     uint64_t callTime) {
    const ReferenceDvGuidanceOutput safeDefault = {
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    if (dvInrtlCmd.squaredNorm() < static_cast<double>(DvGuidanceAlgorithm::kMinNormSq)) {
        return safeDefault;
    }
    const Eigen::Vector3d dvHat_N = dvInrtlCmd.normalized();

    const Eigen::Vector3d cross = dvRotVecUnit.normalized().cross(dvHat_N);
    if (!(cross.squaredNorm() >= static_cast<double>(DvGuidanceAlgorithm::kMinCrossSq))) {
        return safeDefault;
    }
    Eigen::Matrix3d dcm_BubN;
    dcm_BubN.row(0) = dvHat_N;
    dcm_BubN.row(1) = cross.normalized();
    dcm_BubN.row(2) = dcm_BubN.row(0).cross(dcm_BubN.row(1)).normalized();

    const double burnTime =
        static_cast<double>(static_cast<int64_t>(callTime) - static_cast<int64_t>(burnStartTime)) * 1e-9;

    const double angle = dvRotVecMag * burnTime;
    Eigen::Matrix3d dcm_ButBub = Eigen::Matrix3d::Identity();
    if (std::abs(angle) >= static_cast<double>(DvGuidanceAlgorithm::kSmallAngle)) {
        dcm_ButBub = prvToDcm(Eigen::Vector3d{0.0, 0.0, angle});
    }
    const Eigen::Matrix3d dcm_ButN = dcm_ButBub * dcm_BubN;

    return {
        dcm_ButN,
        dvRotVecMag * dcm_ButN.row(2).transpose(),
        Eigen::Vector3d::Zero(),
    };
}

inline void testDvGuidance(const Eigen::Vector3f& dvInrtlCmd,
                           const Eigen::Vector3f& dvRotVecUnit,
                           float dvRotVecMag,
                           uint64_t burnStartTime,
                           uint64_t callTime) {
    DvGuidanceAlgorithm alg(DvGuidanceConfig::create());

    DvGuidanceOutput out;
    EXPECT_NO_THROW(out = alg.update(dvInrtlCmd, dvRotVecUnit, dvRotVecMag, burnStartTime, callTime));

    // Robustness: the output is finite for any input in the domain (the degenerate-input guards
    // return a safe default instead of propagating NaN).
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
    }

    // Accuracy: only compare against the double reference where the inputs are unambiguously on one
    // side of every guard (evaluated in double, with margin) so a FP32-vs-double boundary
    // disagreement can't pit a guarded path against an unguarded one. The guard regions and the
    // snap boundary themselves are covered by explicit edge tests.
    const Eigen::Vector3d cmd_d = dvInrtlCmd.cast<double>();
    const Eigen::Vector3d rot_d = dvRotVecUnit.cast<double>();
    const double sinSq = rot_d.normalized().cross(cmd_d.normalized()).squaredNorm();
    const double burnTime =
        static_cast<double>(static_cast<int64_t>(callTime) - static_cast<int64_t>(burnStartTime)) * 1e-9;
    const double absAngle = std::abs(static_cast<double>(dvRotVecMag) * burnTime);
    const double kSmallAngle = static_cast<double>(DvGuidanceAlgorithm::kSmallAngle);
    const bool nonDegenerate = cmd_d.squaredNorm() > 1.5 * static_cast<double>(DvGuidanceAlgorithm::kMinNormSq) &&
                               sinSq > 1.5 * static_cast<double>(DvGuidanceAlgorithm::kMinCrossSq) &&
                               (absAngle <= 0.5 * kSmallAngle || absAngle >= 2.0 * kSmallAngle);
    if (!nonDegenerate) {
        return;
    }

    ReferenceDvGuidanceOutput ref;
    EXPECT_NO_THROW(ref = referenceDvGuidance(cmd_d, rot_d, static_cast<double>(dvRotVecMag), burnStartTime, callTime));

    constexpr float tol = 1e-5F;
    const Eigen::Matrix3f outDcm = mrpToDcm<float>(out.sigma_RN);
    const Eigen::Matrix3f refDcm = ref.dcm_RN.cast<float>();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(outDcm(i, j), refDcm(i, j), tol);
        }
        EXPECT_NEAR(out.omega_RN_N[i], static_cast<float>(ref.omega_RN_N[i]), tol);
        EXPECT_NEAR(out.domega_RN_N[i], static_cast<float>(ref.domega_RN_N[i]), tol);
    }
}

inline void testDvGuidanceSetup() {
    EXPECT_NO_THROW({
        const DvGuidanceConfig cfg = DvGuidanceConfig::create();
        const DvGuidanceAlgorithm alg(cfg);
        (void)alg;
    });
}

#endif
