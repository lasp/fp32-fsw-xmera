#ifndef TEST_FLYBY_POINT_HELPERS_H
#define TEST_FLYBY_POINT_HELPERS_H

#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "flybyPointAlgorithm.h"
#include "utilities/freestandingIsFinite.hpp"
#include "utilities/safeMath.h"
#include "utilities/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <limits>

struct ReferenceFlybyPointOutput {
    Eigen::Vector3d sigma_RN = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_RN_N = Eigen::Vector3d::Zero();
    Eigen::Vector3d domega_RN_N = Eigen::Vector3d::Zero();
    bool collinearityTrigger = false;
    bool maxRateTrigger = false;
    bool maxAccelerationTrigger = false;
    bool positionKnowledgeExceedTrigger = false;
    bool validOutput = false;
};

struct ReferenceFlybyPointState {
    double dt = 0.0;
    double timeOfFirstRead = 0.0;
    bool firstRead = true;
    double f0 = 0.0;      //!< was float in algorithm — kept double here for reference
    double gamma0 = 0.0;  //!< was float in algorithm — kept double here for reference
    uint64_t lastFilterReadTime = 0;
    Eigen::Matrix3d R0N = Eigen::Matrix3d::Identity();  //!< was Matrix3f in algorithm
    Eigen::Vector3d firstNavPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d firstNavVelocity = Eigen::Vector3d::Zero();
};

/* double-precision reference implementation of the algorithm for numerical comparison */
inline ReferenceFlybyPointOutput referenceFlybyPoint(const FlybyPointConfig& cfg,
                                                     ReferenceFlybyPointState& state,
                                                     uint64_t currentSimNanos,
                                                     const Eigen::Vector3d& r_BN_N,
                                                     const Eigen::Vector3d& v_BN_N) {
    constexpr double eps = std::numeric_limits<double>::epsilon();
    if (r_BN_N.squaredNorm() < eps || v_BN_N.squaredNorm() < eps) {
        FSW_THROW_INVALID_ARGUMENT("inputs r and v must be non-zero");
    }

    bool collinearityTrigger = false;
    bool maxRateTrigger = false;
    bool maxAccelerationTrigger = false;
    bool positionKnowledgeExceedTrigger = false;

    state.dt = static_cast<double>(currentSimNanos - state.lastFilterReadTime) * kNano2Sec;

    if (state.dt >= cfg.getTimeBetweenFilterData() || state.firstRead) {
        const Eigen::Vector3d ur_N = r_BN_N.normalized();
        const Eigen::Vector3d uv_N = v_BN_N.normalized();

        const bool collinear = std::abs(1.0 - ur_N.dot(uv_N)) < static_cast<double>(cfg.getToleranceForCollinearity());

        if (state.firstRead) {
            collinearityTrigger = collinear;
            if (!collinear) {
                state.timeOfFirstRead = static_cast<double>(currentSimNanos) * kNano2Sec;
                state.firstNavPosition = r_BN_N;
                state.firstNavVelocity = v_BN_N;

                // computeFlybyParameters — all double
                state.f0 = v_BN_N.norm() / r_BN_N.norm();
                const Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
                const Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();
                state.gamma0 = safeAtan(v_BN_N.dot(ur_N) / v_BN_N.dot(ut_N));

                // computeRN — double rows
                state.R0N.row(0) = ur_N;
                state.R0N.row(1) = ut_N;
                state.R0N.row(2) = uh_N;

                state.firstRead = false;
            }
        } else {
            collinearityTrigger = collinear;
            bool valid = !collinear;

            const double distanceClosestApproach = -r_BN_N.norm() * safeSin(state.gamma0);
            if (std::abs(distanceClosestApproach) < eps) {
                valid = false;
                maxRateTrigger = true;
                maxAccelerationTrigger = true;
            } else {
                const double maxPredictedRate = v_BN_N.norm() / distanceClosestApproach * 180.0 / M_PI;
                if (maxPredictedRate > static_cast<double>(cfg.getMaxRateThreshold())) {
                    valid = false;
                    maxRateTrigger = true;
                }

                const double angularRateAtCA = v_BN_N.norm() / distanceClosestApproach;
                const double maxPredictedAcceleration =
                    3.0 * safeSqrt(3.0) / 8.0 * angularRateAtCA * angularRateAtCA * 180.0 / M_PI;
                if (maxPredictedAcceleration > static_cast<double>(cfg.getMaxAccelerationThreshold())) {
                    valid = false;
                    maxAccelerationTrigger = true;
                }
            }

            const double deltaT = static_cast<double>(currentSimNanos) * kNano2Sec - state.timeOfFirstRead;
            const double deltaPositionNorm =
                (r_BN_N - (state.firstNavPosition + deltaT * state.firstNavVelocity)).norm();
            if (deltaPositionNorm > static_cast<double>(cfg.getPositionKnowledgeSigma())) {
                valid = false;
                positionKnowledgeExceedTrigger = true;
            }

            if (valid) {
                const Eigen::Vector3d uh_N = ur_N.cross(uv_N).normalized();
                const Eigen::Vector3d ut_N = uh_N.cross(ur_N).normalized();

                state.f0 = v_BN_N.norm() / r_BN_N.norm();
                state.gamma0 = safeAtan(v_BN_N.dot(ur_N) / v_BN_N.dot(ut_N));

                state.R0N.row(0) = ur_N;
                state.R0N.row(1) = ut_N;
                state.R0N.row(2) = uh_N;

                state.lastFilterReadTime = currentSimNanos;
                state.dt = 0.0;
            }
        }
    }

    // computeGuidanceSolution — all double
    const double theta = safeAtan(safeTan(state.gamma0) + state.f0 / safeCos(state.gamma0) * state.dt) - state.gamma0;
    const Eigen::Vector3d PRV_theta{0.0, 0.0, theta};
    const Eigen::Matrix3d RtR0 = prvToDcm(PRV_theta);
    const Eigen::Matrix3d RtN = RtR0 * state.R0N;

    const double den =
        (state.f0 * state.f0 * state.dt * state.dt) + (2.0 * state.f0 * safeSin(state.gamma0) * state.dt) + 1.0;
    const double thetaDot = state.f0 * safeCos(state.gamma0) / den;
    const double thetaDDot = -2.0 * state.f0 * state.f0 * safeCos(state.gamma0) *
                             (state.f0 * state.dt + safeSin(state.gamma0)) / (den * den);
    const Eigen::Vector3d omega_RN_R{0.0, 0.0, thetaDot};
    const Eigen::Vector3d omegaDot_RN_R{0.0, 0.0, thetaDDot};

    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);
    if (cfg.getSignOfOrbitNormalFrameVector() == -1) {
        const Eigen::Vector3d halfRotationX{1.0, 0.0, 0.0};
        sigma_RN = addMrp(sigma_RN, halfRotationX);
    }
    const Eigen::Vector3d omega_RN_N = RtN.transpose() * omega_RN_R;
    const Eigen::Vector3d omegaDot_RN_N = RtN.transpose() * omegaDot_RN_R;

    ReferenceFlybyPointOutput output{};
    output.sigma_RN = sigma_RN;
    output.omega_RN_N = omega_RN_N;
    output.domega_RN_N = omegaDot_RN_N;
    output.collinearityTrigger = collinearityTrigger;
    output.maxRateTrigger = maxRateTrigger;
    output.maxAccelerationTrigger = maxAccelerationTrigger;
    output.positionKnowledgeExceedTrigger = positionKnowledgeExceedTrigger;
    output.validOutput = !state.firstRead && output.sigma_RN.allFinite() && output.omega_RN_N.allFinite() &&
                         output.domega_RN_N.allFinite();
    return output;
}

// -----------------------------------------
// Config Validation Test
// -----------------------------------------
// Verifies that the FlybyPointConfig factory throws on every out-of-range parameter
// and accepts a fully valid configuration.

inline void configValidationTest() {
    EXPECT_NO_THROW({ (void)FlybyPointConfig::create(60.0, 0.01F, 1, 5.0F, 2.0F, 1000.0F); });

    EXPECT_ANY_THROW({ (void)FlybyPointConfig::create(0.0, 0.01F, 1, 5.0F, 2.0F, 1000.0F); });   // timeBetween <= 0
    EXPECT_ANY_THROW({ (void)FlybyPointConfig::create(60.0, 0.0F, 1, 5.0F, 2.0F, 1000.0F); });   // collinTol <= 0
    EXPECT_ANY_THROW({ (void)FlybyPointConfig::create(60.0, 0.01F, 0, 5.0F, 2.0F, 1000.0F); });  // sign not ±1
    EXPECT_ANY_THROW({ (void)FlybyPointConfig::create(60.0, 0.01F, 1, 0.0F, 2.0F, 1000.0F); });  // maxRate <= 0
    EXPECT_ANY_THROW({ (void)FlybyPointConfig::create(60.0, 0.01F, 1, 5.0F, 0.0F, 1000.0F); });  // maxAccel <= 0
    EXPECT_ANY_THROW({ (void)FlybyPointConfig::create(60.0, 0.01F, 1, 5.0F, 2.0F, 0.0F); });     // sigma <= 0
}

// -----------------------------------------
// Regression Test
// -----------------------------------------
// Runs a three-step rectilinear flyby and compares the float algorithm output against the
// double-precision reference at each step:
//   step 0  t = 0                          (first read — seeds the algorithm)
//   step 1  t = 0.5 * timeBetweenFilterData (propagation only, no state update)
//   step 2  t = 1.1 * timeBetweenFilterData (past filter cadence — triggers validity check + update)

inline void regressionTestFlybyPoint(double timeBetweenFilterData,
                                     float toleranceForCollinearity,
                                     int signOfOrbitNormalFrameVector,
                                     float maxRateThreshold,
                                     float maxAccelerationThreshold,
                                     float positionKnowledgeSigma,
                                     const Eigen::Vector3d& r0,
                                     const Eigen::Vector3d& v0) {
    const auto cfg = FlybyPointConfig::create(timeBetweenFilterData,
                                              toleranceForCollinearity,
                                              signOfOrbitNormalFrameVector,
                                              maxRateThreshold,
                                              maxAccelerationThreshold,
                                              positionKnowledgeSigma);
    FlybyPointAlgorithm alg(cfg);
    alg.reset();
    ReferenceFlybyPointState refState{};

    const double tFilter = cfg.getTimeBetweenFilterData();
    const double dt1 = 0.5 * tFilter;
    const double dt2 = 1.1 * tFilter;
    const auto kNano = static_cast<uint64_t>(1.0 / kNano2Sec);
    const uint64_t times[] = {0ULL,
                              static_cast<uint64_t>(dt1 * static_cast<double>(kNano)),
                              static_cast<uint64_t>(dt2 * static_cast<double>(kNano))};
    const Eigen::Vector3d rs[] = {r0, r0 + dt1 * v0, r0 + dt2 * v0};

    for (int step = 0; step < 3; ++step) {
        const FlybyPointOutput out = alg.updateState(times[step], rs[step], v0);
        const ReferenceFlybyPointOutput ref = referenceFlybyPoint(cfg, refState, times[step], rs[step], v0);

        // sigma_RN: compare with MRP shadow to handle equivalent representations
        const Eigen::Vector3d sigma_out = out.sigma_RN.cast<double>();
        Eigen::Vector3d sigma_cmp = ref.sigma_RN;
        if (ref.sigma_RN.squaredNorm() > 1e-12) {
            const Eigen::Vector3d sigma_shadow = -ref.sigma_RN / ref.sigma_RN.squaredNorm();
            if ((sigma_out - sigma_shadow).norm() < (sigma_out - ref.sigma_RN).norm()) {
                sigma_cmp = sigma_shadow;
            }
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(sigma_out[i], sigma_cmp[i], 1e-5);
            EXPECT_NEAR(out.omega_RN_N[i], ref.omega_RN_N[i], 1e-5);
            EXPECT_NEAR(out.domega_RN_N[i], ref.domega_RN_N[i], 1e-5);
        }

        // Diagnostic flags must agree exactly
        EXPECT_EQ(out.collinearityTrigger, ref.collinearityTrigger);
        EXPECT_EQ(out.maxRateTrigger, ref.maxRateTrigger);
        EXPECT_EQ(out.maxAccelerationTrigger, ref.maxAccelerationTrigger);
        EXPECT_EQ(out.positionKnowledgeExceedTrigger, ref.positionKnowledgeExceedTrigger);
        EXPECT_EQ(out.validOutput, ref.validOutput);

        for (int i = 0; i < 3; ++i) {
            EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
            EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
            EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
        }
    }
}

// -----------------------------------------
// Valid Output Seeded Test
// -----------------------------------------
// Verifies that validOutput becomes true after a successful (non-collinear) first read,
// and that reset() followed by a new valid read re-establishes validOutput = true.

inline void validOutputSeededTest(const Eigen::Vector3d& r, const Eigen::Vector3d& v) {
    constexpr double kTimeBetweenFilterData = 60.0;
    const auto cfg = FlybyPointConfig::create(kTimeBetweenFilterData, 0.01F, 1, 5.0F, 2.0F, 1000.0F);
    FlybyPointAlgorithm alg(cfg);
    alg.reset();

    const FlybyPointOutput out = alg.updateState(0ULL, r, v);
    EXPECT_TRUE(out.validOutput);
    EXPECT_FALSE(out.collinearityTrigger);

    alg.reset();
    const FlybyPointOutput out2 = alg.updateState(0ULL, r, v);
    EXPECT_TRUE(out2.validOutput);
}

// -----------------------------------------
// Collinear Rejection Test
// -----------------------------------------
// Verifies that when r and v are parallel (collision trajectory), the algorithm sets
// collinearityTrigger and leaves validOutput false.

inline void collinearRejectionTest() {
    const auto cfg = FlybyPointConfig::create(60.0, 0.01F, 1, 5.0F, 2.0F, 1000.0F);
    FlybyPointAlgorithm alg(cfg);
    alg.reset();

    const Eigen::Vector3d r{1000.0, 0.0, 0.0};
    const Eigen::Vector3d v{500.0, 0.0, 0.0};  // parallel to r

    const FlybyPointOutput out = alg.updateState(0ULL, r, v);
    EXPECT_TRUE(out.collinearityTrigger);
    EXPECT_FALSE(out.validOutput);
}

#endif
