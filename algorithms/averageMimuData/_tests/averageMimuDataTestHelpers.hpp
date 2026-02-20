#ifndef TEST_EPHEMERIDESRECENTER_H
#define TEST_EPHEMERIDESRECENTER_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "averageMimuDataAlgorithm.h"
#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>


OutputAverageAccelAnglevel referenceUpdate(InputPktsData const& localPkts,
                        const AverageMimuDataAlgorithm& alg)
{
    uint64_t maxTimeTag = 0U;
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        maxTimeTag = std::max(localPkts.measTime[i], maxTimeTag);
    }

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        const uint64_t measTime = localPkts.measTime[i];

        // Rolling average with averaging window as window width or the maximum buffer size
        if (static_cast<float>(maxTimeTag - measTime) * NANO2SEC <= alg.getAveragingWindow()) {
            gyroSum_P  += localPkts.gyro_P[i];
            accelSum_P += localPkts.accel_P[i];
            measAvgCount++;
        }
    }

    OutputAverageAccelAnglevel out{};
    if (measAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const gyroSum_B = alg.getDcmPltfToBdy() * gyroSum_P;
        accelSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const accelSum_B = alg.getDcmPltfToBdy() * accelSum_P;
        out.anglevelBody = gyroSum_B;
        out.accelBody = accelSum_B;
    }

    return out;
}

using Vec3Arr = std::array<float, 3>;

inline Eigen::Vector3f toVec3(Vec3Arr const& a) { return Eigen::Vector3f{a[0], a[1], a[2]}; }

struct InputData {
    uint64_t measTime = 0U;
    Vec3Arr gyro_P{{0.0F, 0.0F, 0.0F}};
    Vec3Arr accel_P{{0.0F, 0.0F, 0.0F}};
};

inline void regressionTestaverageMimuData(std::size_t N,
                                          float window,
                                          std::array<InputData, MAX_ACC_BUF_PKT> const& input)
{
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setAveragingWindow(window);

    // Clamp N to valid range
    if (N > MAX_ACC_BUF_PKT) {
        N = MAX_ACC_BUF_PKT;
    }
    // Build the algorithm input buffer (max size), but only populate first N.
    // The rest are neutral so they cannot affect maxTimeTag or the average.
    InputPktsData in{};
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i]   = Eigen::Vector3f::Zero();
        in.accel_P[i]  = Eigen::Vector3f::Zero();
    }

    for (std::size_t i = 0; i < N; ++i) {
        in.measTime[i] = input[i].measTime;
        in.gyro_P[i]   = toVec3(input[i].gyro_P);
        in.accel_P[i]  = toVec3(input[i].accel_P);
    }

    const OutputAverageAccelAnglevel out_alg = alg.update(in);
    const OutputAverageAccelAnglevel out_ref = referenceUpdate(in, alg);

    EXPECT_EQ(out_alg.anglevelBody, out_ref.anglevelBody);
    EXPECT_EQ(out_alg.accelBody, out_ref.accelBody);
}

inline void testKnownSolaverageMimuData() {
    // -----------------------
    // Fixed algorithm settings
    // -----------------------
    AverageMimuDataAlgorithm alg;

    // 90 deg rotation about +Z:
    // [ 0 -1  0
    //   1  0  0
    //   0  0  1 ]
    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f;

    alg.setDcmPltfToBdy(dcm_BP);

    // Time window (seconds)
    // Choose 0.26 so ages 0.00, 0.05, 0.15 are included; 0.30 excluded
    constexpr float window = 0.26f;
    alg.setAveragingWindow(window);

        // -----------------------
    // Fixed synthetic packets (DIRECT)
    // -----------------------
    InputPktsData in{};  // <-- directly build unpacked input

    // zero everything (Eigen::Vector3f default in std::array is uninitialized)
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i]   = Eigen::Vector3f::Zero();
        in.accel_P[i]  = Eigen::Vector3f::Zero();
    }

    // Reference time = 1.0s (ns)
    constexpr uint64_t t_ref = SEC2NANO;

    // Ages in seconds: 0.00, 0.05, 0.15, 0.30 (cleanly away from boundary)
    const uint64_t t0 = t_ref;
    const uint64_t t1 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.05f);
    const uint64_t t2 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.15f);
    const uint64_t t3 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.30f);

    // Choose easy vectors so the mean is simple.
    const Eigen::Vector3f gyro0{ 1.f, 2.f, 3.f};
    const Eigen::Vector3f gyro1{ 3.f, 2.f, 1.f};
    const Eigen::Vector3f gyro2{-1.f, 0.f, 2.f};
    const Eigen::Vector3f gyro3{ 9.f, 9.f, 9.f};  // excluded by averagingWindow

    const Eigen::Vector3f acc0{4.f, 0.f, 0.f};
    const Eigen::Vector3f acc1{0.f, 4.f, 0.f};
    const Eigen::Vector3f acc2{0.f, 0.f, 4.f};
    const Eigen::Vector3f acc3{8.f, 8.f, 8.f};    // excluded by averagingWindow

    // Put packets in the first few slots
    in.measTime[0] = t0;  in.gyro_P[0] = gyro0;  in.accel_P[0] = acc0;
    in.measTime[1] = t1;  in.gyro_P[1] = gyro1;  in.accel_P[1] = acc1;
    in.measTime[2] = t2;  in.gyro_P[2] = gyro2;  in.accel_P[2] = acc2;
    in.measTime[3] = t3;  in.gyro_P[3] = gyro3;  in.accel_P[3] = acc3;

    // -----------------------
    // Run algorithm under test
    // -----------------------
    const OutputAverageAccelAnglevel out_alg = alg.update(in);

    // -----------------------
    // True known solution:
    // include packets 0,1,2 only (ages 0, 0.05, 0.15 < 0.26)
    // mean_P = (v0 + v1 + v2) / 3
    // out_B = dcm_BP * mean_P
    // -----------------------
    const Eigen::Vector3f gyroMean_P = (gyro0 + gyro1 + gyro2) / 3.f;
    const Eigen::Vector3f accMean_P = (acc0 + acc1 + acc2) / 3.f;

    const Eigen::Vector3f gyroTrue_B = dcm_BP * gyroMean_P;
    const Eigen::Vector3f accTrue_B = dcm_BP * accMean_P;

    EXPECT_EQ(out_alg.anglevelBody, gyroTrue_B);
    EXPECT_EQ(out_alg.accelBody, accTrue_B);
}

inline void testZeroAveragingWindow() {
    // -----------------------
    // Fixed algorithm settings
    // -----------------------
    AverageMimuDataAlgorithm alg;

    Eigen::Matrix3f dcm_BP;
    dcm_BP << 0.f, -1.f, 0.f,
              1.f,  0.f, 0.f,
              0.f,  0.f, 1.f;
    alg.setDcmPltfToBdy(dcm_BP);

    // averagingWindow = 0 => should pick the packet(s) at maxTimeTag
    alg.setAveragingWindow(0.0f);

    // -----------------------
    // Fixed synthetic packets (DIRECT)
    // -----------------------
    InputPktsData in{};

    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i]   = Eigen::Vector3f::Zero();
        in.accel_P[i]  = Eigen::Vector3f::Zero();
    }

    constexpr uint64_t t_ref = SEC2NANO;

    // Make packet 0 the unique maxTimeTag
    const uint64_t t0 = t_ref;                                    // max
    const uint64_t t1 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.05f);
    const uint64_t t2 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.15f);
    const uint64_t t3 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.30f);

    const Eigen::Vector3f gyro0{ 1.f, 2.f, 3.f};
    const Eigen::Vector3f gyro1{ 3.f, 2.f, 1.f};
    const Eigen::Vector3f gyro2{-1.f, 0.f, 2.f};
    const Eigen::Vector3f gyro3{ 9.f, 9.f, 9.f};

    const Eigen::Vector3f acc0{4.f, 0.f, 0.f};
    const Eigen::Vector3f acc1{0.f, 4.f, 0.f};
    const Eigen::Vector3f acc2{0.f, 0.f, 4.f};
    const Eigen::Vector3f acc3{8.f, 8.f, 8.f};

    in.measTime[0] = t0;  in.gyro_P[0] = gyro0;  in.accel_P[0] = acc0;
    in.measTime[1] = t1;  in.gyro_P[1] = gyro1;  in.accel_P[1] = acc1;
    in.measTime[2] = t2;  in.gyro_P[2] = gyro2;  in.accel_P[2] = acc2;
    in.measTime[3] = t3;  in.gyro_P[3] = gyro3;  in.accel_P[3] = acc3;

    // -----------------------
    // Run algorithm under test
    // -----------------------
    const OutputAverageAccelAnglevel out_alg = alg.update(in);

    // -----------------------
    // True known solution (averagingWindow = 0):
    // use ONLY the packet with maxTimeTag (packet 0 here)
    // out_B = dcm_BP * v0
    // -----------------------
    const Eigen::Vector3f gyroTrue_B = dcm_BP * gyro0;
    const Eigen::Vector3f accTrue_B  = dcm_BP * acc0;

    EXPECT_EQ(out_alg.anglevelBody, gyroTrue_B);
    EXPECT_EQ(out_alg.accelBody, accTrue_B);
}

inline void testSetupaverageMimuData() {
    AverageMimuDataAlgorithm alg;

    // 1) Setters should not throw
    EXPECT_THROW(alg.setAveragingWindow(-0.1), fs::invalid_argument);
    EXPECT_NO_THROW(alg.setAveragingWindow(0.25f));
    // Not orthonormal (scaling matrix), det != 1 as well
    Eigen::Matrix3f badOrtho = Eigen::Matrix3f::Identity();
    badOrtho(0, 0) = 2.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badOrtho), fs::invalid_argument);
    // det = -1 (reflection), orthonormal but not a proper rotation
    Eigen::Matrix3f badDet = Eigen::Matrix3f::Identity();
    badDet(0, 0) = -1.0F;
    EXPECT_THROW(alg.setDcmPltfToBdy(badDet), fs::invalid_argument);
    EXPECT_NO_THROW(alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity()));

    // 2) Round-trip expectations
    EXPECT_EQ(alg.getAveragingWindow(), 0.25f);
    EXPECT_EQ(alg.getDcmPltfToBdy(), Eigen::Matrix3f::Identity());

    // 3) update() should not throw for a basic input
    InputPktsData in{};
    for (std::size_t i = 0; i < MAX_ACC_BUF_PKT; ++i) {
        in.measTime[i] = 0U;
        in.gyro_P[i]   = Eigen::Vector3f::Zero();
        in.accel_P[i]  = Eigen::Vector3f::Zero();
    }

    // Add one non-zero packet so we exercise the averaging path
    in.measTime[0] = SEC2NANO;
    in.gyro_P[0]   = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    in.accel_P[0]  = Eigen::Vector3f(4.0f, 5.0f, 6.0f);

    EXPECT_NO_THROW((void)alg.update(in));
}

#endif
