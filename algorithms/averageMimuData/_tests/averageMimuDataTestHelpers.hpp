#ifndef TEST_EPHEMERIDESRECENTER_H
#define TEST_EPHEMERIDESRECENTER_H

#include "../freestandingInvalidArgument.h"
#include "architecture/utilities/eigenSupport.h"
#include "averageMimuDataAlgorithm.h"
#include <architecture/utilities/macroDefinitions.h>
#include <gtest/gtest.h>

inline void pushPkt(AccDataMsgF32Payload& msg,
                    std::size_t idx,
                    uint64_t measTime_ns,
                    Eigen::Vector3f const& gyro,
                    Eigen::Vector3f const& accel) {
    auto& pkt = msg.accPkts[idx];
    pkt.measTime = measTime_ns;

    // Assuming pkt.gyro_B and pkt.accel_B are float[3]
    pkt.gyro_B[0] = gyro[0];
    pkt.gyro_B[1] = gyro[1];
    pkt.gyro_B[2] = gyro[2];
    pkt.accel_B[0] = accel[0];
    pkt.accel_B[1] = accel[1];
    pkt.accel_B[2] = accel[2];
}

OutData referenceUpdate(AccDataMsgF32Payload const& localPkts, const AverageMimuDataAlgorithm& alg) {
    uint64_t maxTimeTag = 0U;
    for (auto const& accPkt : localPkts.accPkts) {
        maxTimeTag = std::max(accPkt.measTime, maxTimeTag);
    }

    Eigen::Vector3f gyroSum_P = Eigen::Vector3f::Zero();
    Eigen::Vector3f accelSum_P = Eigen::Vector3f::Zero();
    uint64_t measAvgCount = 0U;

    for (const auto& [measTime, gyro_B, accel_B] : localPkts.accPkts) {
        // Rolling average with timeDelta as window width or the maximum buffer size
        if (static_cast<float>(maxTimeTag - measTime) * NANO2SEC < alg.getTimeDelta()) {
            gyroSum_P += Eigen::Map<const Eigen::Vector3f>(gyro_B);
            accelSum_P += Eigen::Map<const Eigen::Vector3f>(accel_B);
            measAvgCount++;
        }
    }

    OutData out{};
    if (measAvgCount > 0U) {
        gyroSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const gyroSum_B = alg.getDcmPltfToBdy() * gyroSum_P;
        accelSum_P /= static_cast<float>(measAvgCount);
        Eigen::Vector3f const accelSum_B = alg.getDcmPltfToBdy() * accelSum_P;
        out.AngVelBody = gyroSum_B;
        out.AccelBody = accelSum_B;
    }

    return out;
}

using Vec3Arr = std::array<float, 3>;

inline Eigen::Vector3f toVec3(Vec3Arr const& a) { return Eigen::Vector3f{a[0], a[1], a[2]}; }

inline void regressionTestaverageMimuData(float timeDelta,
                                          float time_meas_factor_0,
                                          float time_meas_factor_1,
                                          float time_meas_factor_2,
                                          Vec3Arr const& gyro_0,
                                          Vec3Arr const& accel_0,
                                          Vec3Arr const& gyro_1,
                                          Vec3Arr const& accel_1,
                                          Vec3Arr const& gyro_2,
                                          Vec3Arr const& accel_2,
                                          Vec3Arr const& gyro_3,
                                          Vec3Arr const& accel_3) {
    AverageMimuDataAlgorithm alg;
    alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity());
    alg.setTimeDelta(timeDelta);

    AccDataMsgF32Payload msg{};
    for (auto& pkt : msg.accPkts) {
        pkt.measTime = 0U;
        pkt.gyro_B[0] = pkt.gyro_B[1] = pkt.gyro_B[2] = 0.0f;
        pkt.accel_B[0] = pkt.accel_B[1] = pkt.accel_B[2] = 0.0f;
    }

    constexpr uint64_t t_ref = SEC2NANO;

    // Build Eigen vectors from arrays
    const Eigen::Vector3f g0 = toVec3(gyro_0);
    const Eigen::Vector3f a0 = toVec3(accel_0);
    const Eigen::Vector3f g1 = toVec3(gyro_1);
    const Eigen::Vector3f a1 = toVec3(accel_1);
    const Eigen::Vector3f g2 = toVec3(gyro_2);
    const Eigen::Vector3f a2 = toVec3(accel_2);
    const Eigen::Vector3f g3 = toVec3(gyro_3);
    const Eigen::Vector3f a3 = toVec3(accel_3);

    pushPkt(msg, 0, t_ref, g0, a0);
    pushPkt(msg, 1, t_ref - static_cast<uint64_t>(SEC2NANO * time_meas_factor_0), g1, a1);
    pushPkt(msg, 2, t_ref - static_cast<uint64_t>(SEC2NANO * time_meas_factor_1), g2, a2);
    pushPkt(msg, 3, t_ref - static_cast<uint64_t>(SEC2NANO * time_meas_factor_2), g3, a3);

    const OutData out_alg = alg.update(msg);
    const OutData out_ref = referenceUpdate(msg, alg);

    // Use tolerant comparison for floats
    constexpr float tol = 1e-5f;
    EXPECT_TRUE(out_alg.AngVelBody.isApprox(out_ref.AngVelBody, tol));
    EXPECT_TRUE(out_alg.AccelBody.isApprox(out_ref.AccelBody, tol));
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
    constexpr float timeDelta = 0.26f;
    alg.setTimeDelta(timeDelta);

    // -----------------------
    // Fixed synthetic packets
    // -----------------------
    AccDataMsgF32Payload msg{};
    for (auto& pkt : msg.accPkts) {
        pkt.measTime = 0U;
        pkt.gyro_B[0] = pkt.gyro_B[1] = pkt.gyro_B[2] = 0.0f;
        pkt.accel_B[0] = pkt.accel_B[1] = pkt.accel_B[2] = 0.0f;
    }

    // Reference time = 1.0s (ns)
    constexpr uint64_t t_ref = SEC2NANO;

    // Ages in seconds: 0.00, 0.05, 0.15, 0.30 (cleanly away from boundary)
    const uint64_t t0 = t_ref;
    const uint64_t t1 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.05f);
    const uint64_t t2 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.15f);
    const uint64_t t3 = t_ref - static_cast<uint64_t>(SEC2NANO * 0.30f);

    // Choose easy vectors so the mean is simple.
    const Eigen::Vector3f gyro0{1.f, 2.f, 3.f};
    const Eigen::Vector3f gyro1{3.f, 2.f, 1.f};
    const Eigen::Vector3f gyro2{-1.f, 0.f, 2.f};
    const Eigen::Vector3f gyro3{9.f, 9.f, 9.f};  // excluded by timeDelta

    const Eigen::Vector3f acc0{4.f, 0.f, 0.f};
    const Eigen::Vector3f acc1{0.f, 4.f, 0.f};
    const Eigen::Vector3f acc2{0.f, 0.f, 4.f};
    const Eigen::Vector3f acc3{8.f, 8.f, 8.f};  // excluded by timeDelta

    pushPkt(msg, 0, t0, gyro0, acc0);
    pushPkt(msg, 1, t1, gyro1, acc1);
    pushPkt(msg, 2, t2, gyro2, acc2);
    pushPkt(msg, 3, t3, gyro3, acc3);

    // -----------------------
    // Run algorithm under test
    // -----------------------
    const OutData out_alg = alg.update(msg);

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

    EXPECT_EQ(out_alg.AngVelBody, gyroTrue_B);
    EXPECT_EQ(out_alg.AccelBody, accTrue_B);
}

inline void testSetupaverageMimuData() {
    AverageMimuDataAlgorithm alg;

    // 1) Setters should not throw
    EXPECT_NO_THROW(alg.setTimeDelta(0.25f));
    EXPECT_NO_THROW(alg.setDcmPltfToBdy(Eigen::Matrix3f::Identity()));

    // 2) Round-trip expectations
    EXPECT_FLOAT_EQ(alg.getTimeDelta(), 0.25f);
    EXPECT_TRUE(alg.getDcmPltfToBdy().isApprox(Eigen::Matrix3f::Identity(), 0.0f));

    // 3) update() should not throw for a basic input
    AccDataMsgF32Payload msg{};
    for (auto& pkt : msg.accPkts) {
        pkt.measTime = 0U;
        pkt.gyro_B[0] = pkt.gyro_B[1] = pkt.gyro_B[2] = 0.0f;
        pkt.accel_B[0] = pkt.accel_B[1] = pkt.accel_B[2] = 0.0f;
    }

    // Add one non-zero packet so we exercise the averaging path
    msg.accPkts[0].measTime = SEC2NANO;
    msg.accPkts[0].gyro_B[0] = 1.0f;
    msg.accPkts[0].gyro_B[1] = 2.0f;
    msg.accPkts[0].gyro_B[2] = 3.0f;
    msg.accPkts[0].accel_B[0] = 4.0f;
    msg.accPkts[0].accel_B[1] = 5.0f;
    msg.accPkts[0].accel_B[2] = 6.0f;

    EXPECT_NO_THROW((void)alg.update(msg));
}

#endif
