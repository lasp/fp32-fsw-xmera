#include "rwMotorTorqueTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(RwMotorTorqueTest, ReferenceTest) {
    testRwMotorTorque(Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                      Eigen::Vector3f{0.2F, -0.4F, 0.7F},
                      std::vector<bool>{false, false, true, false},
                      true,
                      true,
                      4,
                      std::vector<float>{0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F, 0.4F, 0.1F, -0.3F, 1.2F},
                      3,
                      std::vector<float>{},
                      std::vector<float>{},
                      0.0F);
}

// Non-zero gain: despin must match the reference and add no net body torque.
TEST(RwMotorTorqueTest, NullSpaceDespinFourWheels) {
    // Four wheels: body x, y, z, and a skewed axis so a 1-D null space exists for 3 control axes.
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{1.0F, 1.0F, 1.0F}.normalized();
    const RwMotorTorqueAvailability availability{};  // all wheels available

    Eigen::Matrix3f controlAxes_B{Eigen::Matrix3f::Identity()};
    constexpr float kOmegaGain = 0.5F;
    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(controlAxes_B, rwConfiguration, availability, kOmegaGain)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<4>() = Eigen::Vector4f{100.0F, -50.0F, 30.0F, 80.0F};
    speeds.rwDesiredSpeeds.head<4>() = Eigen::Vector4f{10.0F, 10.0F, 10.0F, 10.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Vector<float, kMaxNumRw> ref =
        referenceUpdate(controlAxes_B, rwConfiguration, availability, Lr_B, speeds, kOmegaGain);

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(out[i], ref[i], 1e-6);
    }

    // The despin contribution (out - controlOnly) must produce no net body torque (fp32 noise floor relative
    // to the despin magnitude).
    const Eigen::Vector<float, kMaxNumRw> despin = out - controlOnly;
    const Eigen::Vector3f despinBodyTorque = rwConfiguration.GsMatrix_B.leftCols<4>() * despin.head<4>();
    EXPECT_LE(despinBodyTorque.norm(), 1e-6F * despin.norm() + 1e-6F);

    // The despin term must be non-trivial (otherwise the test would pass vacuously).
    EXPECT_GT(despin.norm(), 1e-3);
}

// Zero gain disables despin: output equals the control-only (zero-speed) update.
TEST(RwMotorTorqueTest, NullSpaceNoOpOmegaGainZero) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{1.0F, 1.0F, 1.0F}.normalized();
    const RwMotorTorqueAvailability availability{};

    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, availability, 0.0F)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<4>() = Eigen::Vector4f{100.0F, -50.0F, 30.0F, 80.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> withSpeeds = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(withSpeeds[i], controlOnly[i]);
    }
}

// Three spanning wheels leave no null space, so despin is zero even with a non-zero gain.
TEST(RwMotorTorqueTest, NullSpaceNoOpThreeSpanningWheels) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    const RwMotorTorqueAvailability availability{};

    RwMotorTorqueAlgorithm alg{RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, availability, 0.5F)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<3>() = Eigen::Vector3f{100.0F, -50.0F, 30.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> withSpeeds = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_FLOAT_EQ(withSpeeds[i], controlOnly[i]);
    }
}

// Despin must respect availability: an unavailable wheel gets zero torque, and despin adds no body torque.
TEST(RwMotorTorqueTest, NullSpaceRespectsAvailability) {
    // Five wheels so that, with one unavailable, four available wheels still leave a 1-D null space.
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 5U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{1.0F, 1.0F, 1.0F}.normalized();
    rwConfiguration.GsMatrix_B.col(4) = Eigen::Vector3f{1.0F, -1.0F, 0.5F}.normalized();

    RwMotorTorqueAvailability availability{};
    availability.wheelAvailability[2] = UNAVAILABLE;

    constexpr float kOmegaGain = 0.5F;
    RwMotorTorqueAlgorithm alg{
        RwMotorTorqueConfig::create(makeControlAxes(3U), rwConfiguration, availability, kOmegaGain)};

    RwMotorTorqueSpeeds speeds{};
    speeds.rwSpeeds.head<5>() = Eigen::Matrix<float, 5, 1>{100.0F, -50.0F, 30.0F, 80.0F, -20.0F};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> out = alg.update(Lr_B, speeds);
    const Eigen::Vector<float, kMaxNumRw> controlOnly = alg.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Vector<float, kMaxNumRw> ref =
        referenceUpdate(makeControlAxes(3U), rwConfiguration, availability, Lr_B, speeds, kOmegaGain);

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(out[i], ref[i], 1e-6);
    }

    EXPECT_FLOAT_EQ(out[2], 0.0F);  // unavailable wheel gets no torque

    // The despin term over the available wheels produces no net body torque.
    Eigen::Matrix<float, 3, 5> Gs_avail{Eigen::Matrix<float, 3, 5>::Zero()};
    for (uint32_t i = 0U; i < 5U; ++i) {
        if (i != 2U) {
            Gs_avail.col(i) = rwConfiguration.GsMatrix_B.col(i);
        }
    }
    const Eigen::Vector<float, kMaxNumRw> despin = out - controlOnly;
    const Eigen::Vector3f despinBodyTorque = Gs_avail * despin.head<5>();
    EXPECT_LE(despinBodyTorque.norm(), 1e-6F * despin.norm() + 1e-6F);  // fp32 noise floor
    EXPECT_GT(despin.norm(), 1e-3);                                     // despin is non-trivial
}

TEST(RwMotorTorqueTest, SetupTest) { testRwMotorTorqueSetup(); }

// Control axes may sit in any rows; only the spanned subspace matters. Controlling body x and z via
// non-contiguous rows {0, 2} must produce the same motor torques as the contiguous rows {0, 1}.
TEST(RwMotorTorqueTest, NonContiguousControlAxes) {
    // Three wheels spinning about body x, y, z so any control axis is reachable.
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};
    const RwMotorTorqueAvailability availability{};  // all wheels available

    Eigen::Matrix3f contiguous{Eigen::Matrix3f::Zero()};
    contiguous.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    contiguous.row(1) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    Eigen::Matrix3f nonContiguous{Eigen::Matrix3f::Zero()};
    nonContiguous.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    nonContiguous.row(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    RwMotorTorqueAlgorithm algContiguous{RwMotorTorqueConfig::create(contiguous, rwConfiguration, availability)};
    RwMotorTorqueAlgorithm algNonContiguous{RwMotorTorqueConfig::create(nonContiguous, rwConfiguration, availability)};

    const Eigen::Vector3f Lr_B{0.3F, -0.5F, 0.8F};
    const Eigen::Vector<float, kMaxNumRw> outContiguous = algContiguous.update(Lr_B, RwMotorTorqueSpeeds{});
    const Eigen::Vector<float, kMaxNumRw> outNonContiguous = algNonContiguous.update(Lr_B, RwMotorTorqueSpeeds{});

    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        EXPECT_NEAR(outContiguous[i], outNonContiguous[i], 1e-6);
    }
}

// Control axes that are unit but slightly non-orthogonal (within the validation tolerance) are stored
// orthonormalized, and zero (uncontrolled) rows stay zero.
TEST(RwMotorTorqueTest, ControlAxesAreOrthonormalized) {
    Eigen::Matrix3f controlAxes{Eigen::Matrix3f::Zero()};
    controlAxes.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes.row(1) = Eigen::Vector3f{8e-4F, 1.0F, 0.0F}.normalized();  // unit, ~8e-4 off orthogonal to row 0

    // Three wheels spanning body x, y, z so create() accepts the (controllable) configuration.
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 3U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{0.0F, 0.0F, 1.0F};

    const RwMotorTorqueConfig config =
        RwMotorTorqueConfig::create(controlAxes, rwConfiguration, RwMotorTorqueAvailability{});
    const Eigen::Matrix3f& stored = config.getControlAxes();

    EXPECT_NEAR(stored.row(0).norm(), 1.0F, 1e-6);
    EXPECT_NEAR(stored.row(1).norm(), 1.0F, 1e-6);
    EXPECT_NEAR(stored.row(0).dot(stored.row(1)), 0.0F, 1e-6);  // exactly orthogonal after Gram-Schmidt
    EXPECT_NEAR(stored.row(2).norm(), 0.0F, 1e-6);              // uncontrolled row untouched
}

// A near-degenerate control geometry (two wheels nearly aligned, so the second control axis is barely
// reachable) is rejected by create(): cond([CGs]) exceeds the limit even though the mapping is full rank.
TEST(RwMotorTorqueTest, IllConditionedControlMappingRejected) {
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 2U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{1.0F, 5e-3F, 0.0F}.normalized();  // ~0.3 deg off wheel 0

    // Control body x and y: y is reachable only through the tiny y-component of the near-parallel wheels.
    Eigen::Matrix3f controlAxes{Eigen::Matrix3f::Zero()};
    controlAxes.row(0) = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
    controlAxes.row(1) = Eigen::Vector3f{0.0F, 1.0F, 0.0F};

    EXPECT_THROW(RwMotorTorqueConfig::create(controlAxes, rwConfiguration, RwMotorTorqueAvailability{}),
                 fsw::invalid_argument);
}

// Four near-coplanar wheels: the control mapping (body x, y) is well-conditioned, but the null-space (despin)
// geometry is ill-conditioned (cond([Gs]) > 100), so create() rejects the configuration regardless of gain.
TEST(RwMotorTorqueTest, IllConditionedDespinGeometryRejected) {
    constexpr float kOutOfPlane = 1e-3F;  // tiny z component -> [Gs] barely spans the third dimension
    RwMotorTorqueArrayConfiguration rwConfiguration{};
    rwConfiguration.numRW = 4U;
    rwConfiguration.GsMatrix_B.col(0) = Eigen::Vector3f{1.0F, 0.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 1.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(2) = Eigen::Vector3f{-1.0F, 0.0F, kOutOfPlane}.normalized();
    rwConfiguration.GsMatrix_B.col(3) = Eigen::Vector3f{0.0F, -1.0F, kOutOfPlane}.normalized();

    EXPECT_THROW(RwMotorTorqueConfig::create(makeControlAxes(2U), rwConfiguration, RwMotorTorqueAvailability{}, 0.5F),
                 fsw::invalid_argument);
}
