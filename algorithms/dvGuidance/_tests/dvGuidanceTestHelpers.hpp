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
// so the test comparison is robust at the MRP shadow-set boundary (|sigma|=1 at 180 deg).
inline ReferenceDvGuidanceOutput referenceDvGuidance(const Eigen::Vector3d& dvInrtlCmd,
                                                     const Eigen::Vector3d& dvRotVecUnit,
                                                     double dvRotVecMag,
                                                     uint64_t burnStartTime,
                                                     uint64_t callTime) {
    const Eigen::Vector3d dvHat_N = dvInrtlCmd.normalized();
    Eigen::Matrix3d dcm_BubN;
    dcm_BubN.row(0) = dvHat_N;
    dcm_BubN.row(1) = dvRotVecUnit.cross(dvHat_N).normalized();
    dcm_BubN.row(2) = dcm_BubN.row(0).cross(dcm_BubN.row(1)).normalized();

    const double burnTime =
        static_cast<double>(static_cast<int64_t>(callTime) - static_cast<int64_t>(burnStartTime)) * 1e-9;

    const Eigen::Vector3d prv = {0.0, 0.0, dvRotVecMag * burnTime};
    const Eigen::Matrix3d dcm_ButBub = prvToDcm(prv);
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

    ReferenceDvGuidanceOutput ref;
    EXPECT_NO_THROW(ref = referenceDvGuidance(dvInrtlCmd.cast<double>(),
                                              dvRotVecUnit.cast<double>(),
                                              static_cast<double>(dvRotVecMag),
                                              burnStartTime,
                                              callTime));

    constexpr float tol = 1e-5F;
    const Eigen::Matrix3f outDcm = mrpToDcm<float>(out.sigma_RN);
    const Eigen::Matrix3f refDcm = ref.dcm_RN.cast<float>();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(outDcm(i, j), refDcm(i, j), tol);
        }
        EXPECT_NEAR(out.omega_RN_N[i], static_cast<float>(ref.omega_RN_N[i]), tol);
        EXPECT_NEAR(out.domega_RN_N[i], static_cast<float>(ref.domega_RN_N[i]), tol);

        EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
        EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
        EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
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
