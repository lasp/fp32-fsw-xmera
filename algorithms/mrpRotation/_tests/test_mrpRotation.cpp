#include "mrpRotationTestHelpers.hpp"

#include <numbers>

namespace {
constexpr float kDeg = std::numbers::pi_v<float> / 180.0F;

// Nominal mounting of a camera whose boresight +Z_C is -Y_B: rows are the camera axes in body components.
Eigen::Vector3f cameraNominalMounting() {
    Eigen::Matrix3f dcm_CnB;
    dcm_CnB << 0.0F, 0.0F, 1.0F, -1.0F, 0.0F, 0.0F, 0.0F, -1.0F, 0.0F;
    return dcmToMrp(dcm_CnB);
}

// The same camera as measured: the nominal frame turned 0.5 deg about a skewed axis.
Eigen::Vector3f cameraMeasuredMounting() {
    const Eigen::Matrix3f dcm_CCn = mrpToDcm(mrpFromAxisAngle(Eigen::Vector3f{1.0F, -2.0F, 0.5F}, 0.5F * kDeg));
    return dcmToMrp(Eigen::Matrix3f(dcm_CCn * mrpToDcm(cameraNominalMounting())));
}

// Base frame that puts body -Y on the input reference x-axis (the target) and body +X on the input reference z-axis.
Eigen::Vector3f baseMinusYOnTarget() {
    Eigen::Matrix3f dcm_RbR0;
    dcm_RbR0 << 0.0F, 0.0F, 1.0F, -1.0F, 0.0F, 0.0F, 0.0F, -1.0F, 0.0F;
    return dcmToMrp(dcm_RbR0);
}
}  // namespace

// ---------------------------------------------------------------------------
// Regression test: drive the algorithm through several integration steps and compare
// to the reference implementation.
// ---------------------------------------------------------------------------

TEST(MrpRotationTest, RegressionStaticFrame) {
    regressionTestMrpRotation(/* sigma_RbR0 = */ Eigen::Vector3f{0.3F, 0.5F, 0.0F},
                              /* sigma_CnB = */ cameraNominalMounting(),
                              /* sigma_CB = */ cameraMeasuredMounting(),
                              /* sigma_RpRc_C = */ Eigen::Vector3f{0.05F, 0.0F, -0.02F},
                              /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                              /* spinRate = */ 0.0017453F,
                              /* sigma_R0N = */ Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                              /* omega_R0N_N = */ Eigen::Vector3f::Zero(),
                              /* domega_R0N_N = */ Eigen::Vector3f::Zero(),
                              /* updateTimeSec = */ 0.5F,
                              /* numSteps = */ 4);
}

TEST(MrpRotationTest, RegressionSpinningInputFrame) {
    regressionTestMrpRotation(/* sigma_RbR0 = */ Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                              /* sigma_CnB = */ Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                              /* sigma_CB = */ Eigen::Vector3f{0.002F, -0.001F, 0.0F},
                              /* sigma_RpRc_C = */ Eigen::Vector3f{0.0F, 0.1F, 0.0F},
                              /* spinAxis_C = */ Eigen::Vector3f{0.01F, -0.005F, 0.002F}.stableNormalized(),
                              /* spinRate = */ -0.0113578F,
                              /* sigma_R0N = */ Eigen::Vector3f{0.05F, 0.05F, 0.0F},
                              /* omega_R0N_N = */ Eigen::Vector3f{0.1F, 0.0F, 0.0F},
                              /* domega_R0N_N = */ Eigen::Vector3f::Zero(),
                              /* updateTimeSec = */ 0.25F,
                              /* numSteps = */ 6);
}

// ---------------------------------------------------------------------------
// Setup tests: Config validators and constructor behavior.
// ---------------------------------------------------------------------------

TEST(MrpRotationConfigTest, RejectsNonFiniteSigma_RbR0) {
    const Eigen::Vector3f bad{std::nanf(""), 0.0F, 0.0F};
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(
        MrpRotationConfig::create(bad, good, good, good, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
        fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteSigma_CnB) {
    const Eigen::Vector3f bad{0.0F, std::numeric_limits<float>::infinity(), 0.0F};
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(
        MrpRotationConfig::create(good, bad, good, good, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
        fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteSigma_CB) {
    const Eigen::Vector3f bad{0.0F, 0.0F, std::nanf("")};
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(
        MrpRotationConfig::create(good, good, bad, good, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
        fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteSigma_RpRc_C) {
    const Eigen::Vector3f bad{0.0F, 0.0F, std::numeric_limits<float>::infinity()};
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(
        MrpRotationConfig::create(good, good, good, bad, axis, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
        fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonFiniteOrNonUnitSpinAxis) {
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f badAxes[] = {
        Eigen::Vector3f::Zero(),
        1.002F * Eigen::Vector3f::UnitY(),
        0.998F * Eigen::Vector3f::UnitY(),
        Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F},
        Eigen::Vector3f{0.0F, std::numeric_limits<float>::infinity(), 0.0F},
    };
    for (const Eigen::Vector3f& bad : badAxes) {
        EXPECT_THROW(
            MrpRotationConfig::create(good, good, good, good, bad, /* spinRate = */ 0.0F, /* controlPeriod = */ 0.5F),
            fsw::invalid_argument);
    }
}

TEST(MrpRotationConfigTest, RejectsNonFiniteSpinRate) {
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    EXPECT_THROW(MrpRotationConfig::create(good, good, good, good, axis, std::nanf(""), /* controlPeriod = */ 0.5F),
                 fsw::invalid_argument);
    EXPECT_THROW(MrpRotationConfig::create(good,
                                           good,
                                           good,
                                           good,
                                           axis,
                                           -std::numeric_limits<float>::infinity(),
                                           /* controlPeriod = */ 0.5F),
                 fsw::invalid_argument);
}

TEST(MrpRotationConfigTest, RejectsNonPositiveControlPeriod) {
    const Eigen::Vector3f good = Eigen::Vector3f::Zero();
    const Eigen::Vector3f axis = Eigen::Vector3f::UnitX();
    for (const float badPeriod : {0.0F, -0.1F, std::nanf("")}) {
        EXPECT_THROW(MrpRotationConfig::create(good, good, good, good, axis, /* spinRate = */ 0.0F, badPeriod),
                     fsw::invalid_argument);
    }
}

TEST(MrpRotationConfigTest, AcceptsFiniteInputs) {
    EXPECT_NO_THROW(MrpRotationConfig::create(/* sigma_RbR0 = */ Eigen::Vector3f{0.1F, 0.2F, 0.3F},
                                              /* sigma_CnB = */ cameraNominalMounting(),
                                              /* sigma_CB = */ cameraMeasuredMounting(),
                                              /* sigma_RpRc_C = */ Eigen::Vector3f{-0.1F, 0.0F, 0.05F},
                                              /* spinAxis_C = */ Eigen::Vector3f{0.6F, 0.0F, -0.8F},
                                              /* spinRate = */ -0.02F,
                                              /* controlPeriod = */ 0.5F));
}

TEST(MrpRotationConfigTest, GettersRoundTrip) {
    const Eigen::Vector3f sigma_RbR0{0.1F, 0.2F, 0.3F};
    const Eigen::Vector3f sigma_CnB{0.0F, -0.4F, 0.1F};
    const Eigen::Vector3f sigma_CB{0.01F, -0.41F, 0.1F};
    const Eigen::Vector3f sigma_RpRc_C{-0.1F, 0.0F, 0.05F};
    const Eigen::Vector3f spinAxis_C{0.0F, 0.6F, 0.8F};
    constexpr float spinRate = -0.03F;
    constexpr float controlPeriod = 0.25F;
    const auto cfg =
        MrpRotationConfig::create(sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C, spinAxis_C, spinRate, controlPeriod);

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getSigma_RbR0()(i), sigma_RbR0(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_CnB()(i), sigma_CnB(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_CB()(i), sigma_CB(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_RpRc_C()(i), sigma_RpRc_C(i));
        EXPECT_FLOAT_EQ(cfg.getSpinAxis_C()(i), spinAxis_C(i));
    }
    EXPECT_FLOAT_EQ(cfg.getSpinRate(), spinRate);
    EXPECT_FLOAT_EQ(cfg.getControlPeriod(), controlPeriod);
}

// create() accepts a spin axis whose norm is within 1e-3 of 1 and stores it normalized.
TEST(MrpRotationConfigTest, SpinAxisStoredNormalized) {
    const Eigen::Vector3f direction = Eigen::Vector3f{1.0F, -2.0F, 2.0F}.stableNormalized();
    const Eigen::Vector3f nearUnitAxis = 1.0009F * direction;
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto cfg = MrpRotationConfig::create(
        zero, zero, zero, zero, nearUnitAxis, /* spinRate = */ 0.01F, /* controlPeriod = */ 0.5F);

    EXPECT_NEAR(cfg.getSpinAxis_C().stableNorm(), 1.0F, 1e-6F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(cfg.getSpinAxis_C()(i), direction(i), 1e-6F);
    }
}

// create() bounds every MRP to the principal set: an MRP with norm > 1 is stored as its shadow-set
// representative (-sigma / |sigma|^2), which has norm <= 1.
TEST(MrpRotationConfigTest, OffsetsSwitchedToShadowSetWhenNormExceedsOne) {
    const Eigen::Vector3f largeSigma{0.8F, 0.6F, 0.6F};  // |sigma|^2 = 1.36 > 1
    ASSERT_GT(largeSigma.stableNorm(), 1.0F) << "Test setup: MRP must exceed the norm-1 boundary";
    const Eigen::Vector3f expectedShadow = -largeSigma / largeSigma.squaredNorm();

    const auto cfg = MrpRotationConfig::create(largeSigma,
                                               largeSigma,
                                               largeSigma,
                                               largeSigma,
                                               /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 0.5F);

    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getSigma_RbR0()(i), expectedShadow(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_CnB()(i), expectedShadow(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_CB()(i), expectedShadow(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_RpRc_C()(i), expectedShadow(i));
    }
}

// MRPs already within the principal set (norm <= 1) are stored unchanged.
TEST(MrpRotationConfigTest, OffsetsWithinBoundStoredUnchanged) {
    const Eigen::Vector3f sigma_RbR0{0.3F, -0.4F, 0.2F};
    const Eigen::Vector3f sigma_CnB = cameraNominalMounting();
    const Eigen::Vector3f sigma_CB = cameraMeasuredMounting();
    const Eigen::Vector3f sigma_RpRc_C{-0.1F, 0.2F, 0.0F};
    for (const Eigen::Vector3f& sigma : {sigma_RbR0, sigma_CnB, sigma_CB, sigma_RpRc_C}) {
        ASSERT_LE(sigma.stableNorm(), 1.0F);
    }

    const auto cfg = MrpRotationConfig::create(sigma_RbR0,
                                               sigma_CnB,
                                               sigma_CB,
                                               sigma_RpRc_C,
                                               /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 0.5F);
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(cfg.getSigma_RbR0()(i), sigma_RbR0(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_CnB()(i), sigma_CnB(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_CB()(i), sigma_CB(i));
        EXPECT_FLOAT_EQ(cfg.getSigma_RpRc_C()(i), sigma_RpRc_C(i));
    }
}

TEST(MrpRotationConfigTest, IsValidValidatorsHonorContracts) {
    for (const auto isValidSigma : {&MrpRotationConfig::isValidSigma_RbR0,
                                    &MrpRotationConfig::isValidSigma_CnB,
                                    &MrpRotationConfig::isValidSigma_CB,
                                    &MrpRotationConfig::isValidSigma_RpRc_C}) {
        EXPECT_TRUE(isValidSigma(Eigen::Vector3f{0.0F, 0.0F, 0.0F}));
        EXPECT_TRUE(isValidSigma(Eigen::Vector3f{1.5F, -2.0F, 3.5F}));
        EXPECT_FALSE(isValidSigma(Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F}));
        EXPECT_FALSE(isValidSigma(Eigen::Vector3f{0.0F, std::numeric_limits<float>::infinity(), 0.0F}));
    }

    EXPECT_TRUE(MrpRotationConfig::isValidSpinAxis_C(Eigen::Vector3f::UnitZ()));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinAxis_C(1.0009F * Eigen::Vector3f::UnitZ()));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinAxis_C(0.9991F * Eigen::Vector3f::UnitZ()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_C(1.0011F * Eigen::Vector3f::UnitZ()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_C(0.9989F * Eigen::Vector3f::UnitZ()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_C(Eigen::Vector3f::Zero()));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinAxis_C(Eigen::Vector3f{std::nanf(""), 0.0F, 0.0F}));
    EXPECT_FALSE(
        MrpRotationConfig::isValidSpinAxis_C(Eigen::Vector3f{0.0F, 0.0F, std::numeric_limits<float>::infinity()}));

    EXPECT_TRUE(MrpRotationConfig::isValidSpinRate(0.0F));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinRate(0.5F));
    EXPECT_TRUE(MrpRotationConfig::isValidSpinRate(-0.5F));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinRate(std::nanf("")));
    EXPECT_FALSE(MrpRotationConfig::isValidSpinRate(std::numeric_limits<float>::infinity()));

    EXPECT_TRUE(MrpRotationConfig::isValidControlPeriod(0.5F));
    EXPECT_TRUE(MrpRotationConfig::isValidControlPeriod(1e-6F));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(0.0F));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(-1.0F));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(std::nanf("")));
    EXPECT_FALSE(MrpRotationConfig::isValidControlPeriod(std::numeric_limits<float>::infinity()));
}

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

TEST(MrpRotationTest, OutputIsFiniteSmallInputs) {
    propertyOutputIsFinite(/* sigma_RbR0 = */ {0.1F, -0.2F, 0.3F},
                           /* sigma_CnB = */ cameraNominalMounting(),
                           /* sigma_CB = */ cameraMeasuredMounting(),
                           /* sigma_RpRc_C = */ {0.0F, 0.05F, 0.0F},
                           /* spinAxis_C = */ {1.0F, 0.0F, 0.0F},
                           /* spinRate = */ 0.01F);
}

TEST(MrpRotationTest, OutputIsFiniteLargeOmega) {
    propertyOutputIsFinite(/* sigma_RbR0 = */ {0.0F, 0.0F, 0.0F},
                           /* sigma_CnB = */ {0.0F, 0.0F, 0.0F},
                           /* sigma_CB = */ {0.0F, 0.0F, 0.0F},
                           /* sigma_RpRc_C = */ {0.0F, 0.0F, 0.0F},
                           /* spinAxis_C = */ Eigen::Vector3f{5.0F, -3.0F, 2.0F}.stableNormalized(),
                           /* spinRate = */ 6.164414F);
}

TEST(MrpRotationTest, SigmaRNEqualsSpinWhenFixedRotationsAndInputRefAreIdentity) {
    propertySigmaRNEqualsSpinWhenFixedRotationsAndInputRefAreIdentity(
        /* spinAxis_C = */ Eigen::Vector3f{0.05F, -0.02F, 0.01F}.stableNormalized(), /* spinRate = */ 0.054772F);
}

TEST(MrpRotationTest, SigmaRNNormLessOrEqualToOne) {
    // The rate is large enough that the 100-step run crosses the |sigma| = 1 shadow-switch boundary several times.
    propertySigmaRNNormLessOrEqualToOne(/* sigma_RbR0 = */ {0.5F, 0.0F, 0.0F},
                                        /* sigma_CnB = */ cameraNominalMounting(),
                                        /* sigma_CB = */ cameraMeasuredMounting(),
                                        /* sigma_RpRc_C = */ {0.0F, 0.1F, 0.0F},
                                        /* spinAxis_C = */ Eigen::Vector3f{0.3F, 0.1F, -0.2F}.stableNormalized(),
                                        /* spinRate = */ 0.374166F);
}

TEST(MrpRotationTest, OmegaRNDecomposesCorrectly) {
    propertyOmegaRNDecomposesCorrectly(/* sigma_RbR0 = */ {0.1F, 0.2F, -0.1F},
                                       /* sigma_CnB = */ cameraNominalMounting(),
                                       /* sigma_CB = */ cameraMeasuredMounting(),
                                       /* sigma_RpRc_C = */ {0.0F, -0.05F, 0.02F},
                                       /* spinAxis_C = */ Eigen::Vector3f{0.05F, -0.03F, 0.02F}.stableNormalized(),
                                       /* spinRate = */ -0.061644F,
                                       /* sigma_R0N = */ {0.05F, 0.1F, 0.0F},
                                       /* omega_R0N_N = */ {0.02F, 0.0F, -0.01F});
}

TEST(MrpRotationTest, OutputRefEqualsInputRefWhenRotationIsZero) {
    propertyOutputRefEqualsInputRefWhenRotationIsZero(/* sigma_R0N = */ {0.1F, 0.2F, -0.3F},
                                                      /* omega_R0N_N = */ {0.05F, -0.02F, 0.01F},
                                                      /* domega_R0N_N = */ {0.0F, 0.03F, -0.01F});
}

TEST(MrpRotationTest, OutputEqualsOffsetInputRefWhenRateIsZero) {
    propertyOutputEqualsOffsetInputRefWhenRateIsZero(/* sigma_RbR0 = */ {0.3F, -0.4F, 0.2F},
                                                     /* sigma_CnB = */ cameraNominalMounting(),
                                                     /* sigma_CB = */ cameraMeasuredMounting(),
                                                     /* sigma_RpRc_C = */ {0.0F, 0.0F, 0.0437F},
                                                     /* sigma_R0N = */ {0.1F, 0.2F, -0.3F});
}

TEST(MrpRotationTest, ComponentFrameSitsOnNominalFrameAtBase) {
    propertyComponentFrameSitsOnNominalFrameAtBase(/* sigma_RbR0 = */ baseMinusYOnTarget(),
                                                   /* sigma_CnB = */ cameraNominalMounting(),
                                                   /* sigma_CB = */ cameraMeasuredMounting(),
                                                   /* sigma_R0N = */ {0.1F, 0.2F, -0.3F});
}

TEST(MrpRotationTest, RotationAxisFixedInInputRef) {
    propertyRotationAxisFixedInInputRef(/* sigma_RbR0 = */ {0.3F, -0.4F, 0.2F},
                                        /* sigma_CnB = */ cameraNominalMounting(),
                                        /* sigma_CB = */ cameraMeasuredMounting(),
                                        /* sigma_RpRc_C = */ {0.0437F, 0.0F, 0.0F},
                                        /* spinAxis_C = */ {1.0F, 0.0F, 0.0F},
                                        /* spinRate = */ 0.05F);
}

// ---------------------------------------------------------------------------
// Component pointing tests
// ---------------------------------------------------------------------------

// With the measured mounting 0.5 deg off the nominal one and no offset or spin, the body turns so that the
// measured boresight +Z_C, not the nominal body axis -Y_B, lands on the target. The nominal axis is then off the
// target by the angle between the nominal and measured boresights.
TEST(MrpRotationTest, MeasuredMountingPutsRealBoresightOnTarget) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto cfg = MrpRotationConfig::create(baseMinusYOnTarget(),
                                               cameraNominalMounting(),
                                               cameraMeasuredMounting(),
                                               zero,
                                               /* spinAxis_C = */ Eigen::Vector3f::UnitZ(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 0.5F);
    MrpRotationAlgorithm alg{cfg};
    const MrpRotationOutput out = alg.update(MrpRotationAttRefInputs{});

    const Eigen::Vector3f target_R0 = Eigen::Vector3f::UnitX();
    const Eigen::Matrix3f dcm_R0B = mrpToDcm(out.sigma_RN).transpose();  // identity input reference: N = R0
    const Eigen::Vector3f boresight_B = mrpToDcm(cameraMeasuredMounting()).transpose() * Eigen::Vector3f::UnitZ();
    EXPECT_LT(angleBetween(dcm_R0B * boresight_B, target_R0), 1e-5F);
    const float boresightMisalignment = angleBetween(boresight_B, -Eigen::Vector3f::UnitY());
    EXPECT_GT(boresightMisalignment, 0.4F * kDeg) << "Test setup: the measured boresight must be visibly misaligned";
    EXPECT_NEAR(angleBetween(dcm_R0B * -Eigen::Vector3f::UnitY(), target_R0), boresightMisalignment, 1e-5F);
}

// The offset is about component axes: a positive rotation of 10 deg about the camera x-axis moves the boresight
// 10 deg off the target, toward the nominal camera -y axis.
TEST(MrpRotationTest, OffsetInComponentAxesTiltsBoresightOffTarget) {
    const float offsetAngle = 10.0F * kDeg;
    const auto cfg = MrpRotationConfig::create(baseMinusYOnTarget(),
                                               cameraNominalMounting(),
                                               cameraNominalMounting(),
                                               mrpFromAxisAngle(Eigen::Vector3f::UnitX(), offsetAngle),
                                               /* spinAxis_C = */ Eigen::Vector3f::UnitZ(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 0.5F);
    MrpRotationAlgorithm alg{cfg};
    const MrpRotationOutput out = alg.update(MrpRotationAttRefInputs{});

    // The camera frame wrt R0 with the body on Rc, before the offset.
    const Eigen::Matrix3f dcm_CR0 = mrpToDcm(cameraNominalMounting()) * mrpToDcm(baseMinusYOnTarget());
    const Eigen::Vector3f target_R0 = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f minusYCamera_R0 = dcm_CR0.transpose() * -Eigen::Vector3f::UnitY();
    const Eigen::Vector3f boresight_R0 =
        mrpToDcm(out.sigma_RN).transpose() * (mrpToDcm(cameraNominalMounting()).transpose() * Eigen::Vector3f::UnitZ());

    EXPECT_NEAR(angleBetween(boresight_R0, target_R0), offsetAngle, 1e-5F);
    EXPECT_NEAR(boresight_R0.dot(minusYCamera_R0), std::sin(offsetAngle), 1e-5F);
}

// The offset comes after the component: with the measured mounting 0.5 deg off the nominal one, the attitude with the
// offset is the component-only attitude turned by the offset angle about the measured camera x axis.
TEST(MrpRotationTest, OffsetComesAfterTheComponent) {
    const float offsetAngle = 10.0F * kDeg;
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto componentOnly = MrpRotationConfig::create(baseMinusYOnTarget(),
                                                         cameraNominalMounting(),
                                                         cameraMeasuredMounting(),
                                                         zero,
                                                         /* spinAxis_C = */ Eigen::Vector3f::UnitZ(),
                                                         /* spinRate = */ 0.0F,
                                                         /* controlPeriod = */ 0.5F);
    const auto withOffset = MrpRotationConfig::create(baseMinusYOnTarget(),
                                                      cameraNominalMounting(),
                                                      cameraMeasuredMounting(),
                                                      mrpFromAxisAngle(Eigen::Vector3f::UnitX(), offsetAngle),
                                                      /* spinAxis_C = */ Eigen::Vector3f::UnitZ(),
                                                      /* spinRate = */ 0.0F,
                                                      /* controlPeriod = */ 0.5F);
    const MrpRotationAttRefInputs attRef{Eigen::Vector3f{0.1F, 0.2F, -0.3F}, zero, zero};
    const Eigen::Matrix3f dcm_RcN = mrpToDcm(MrpRotationAlgorithm{componentOnly}.update(attRef).sigma_RN);
    const Eigen::Matrix3f dcm_RpN = mrpToDcm(MrpRotationAlgorithm{withOffset}.update(attRef).sigma_RN);

    const Eigen::Vector3f cameraX_B = mrpToDcm(cameraMeasuredMounting()).transpose() * Eigen::Vector3f::UnitX();
    const Eigen::Matrix3f expectedDcm_RpRc = mrpToDcm(mrpFromAxisAngle(cameraX_B, offsetAngle));
    EXPECT_LT(dcmMaxAbsDiff(dcm_RpN * dcm_RcN.transpose(), expectedDcm_RpRc), 1e-5F);
}

// The spin comes after the offset, so a spin about the boresight +Z_C holds the offset boresight still and rolls the
// spacecraft about it: the camera x-axis turns by the spin angle while the boresight does not move.
TEST(MrpRotationTest, SpinAboutBoresightHoldsOffsetBoresight) {
    const float offsetAngle = 10.0F * kDeg;
    constexpr float kRate = 1.0F * kDeg;
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 180;  // 90 deg of spin
    const auto cfg = MrpRotationConfig::create(baseMinusYOnTarget(),
                                               cameraNominalMounting(),
                                               cameraMeasuredMounting(),
                                               mrpFromAxisAngle(Eigen::Vector3f::UnitX(), offsetAngle),
                                               /* spinAxis_C = */ Eigen::Vector3f::UnitZ(),
                                               kRate,
                                               kControlPeriod);
    MrpRotationAlgorithm alg{cfg};

    const Eigen::Matrix3f dcm_BC = mrpToDcm(cameraMeasuredMounting()).transpose();
    const Eigen::Vector3f target_R0 = Eigen::Vector3f::UnitX();
    Eigen::Vector3f firstBoresight_R0 = Eigen::Vector3f::Zero();
    Eigen::Vector3f firstCameraX_R0 = Eigen::Vector3f::Zero();
    Eigen::Vector3f lastCameraX_R0 = Eigen::Vector3f::Zero();
    float maxBoresightDrift = 0.0F;
    for (int k = 0; k < kNumSteps; ++k) {
        const Eigen::Matrix3f dcm_R0B = mrpToDcm(alg.update(MrpRotationAttRefInputs{}).sigma_RN).transpose();
        const Eigen::Vector3f boresight_R0 = dcm_R0B * dcm_BC * Eigen::Vector3f::UnitZ();
        lastCameraX_R0 = dcm_R0B * dcm_BC * Eigen::Vector3f::UnitX();
        if (k == 0) {
            firstBoresight_R0 = boresight_R0;
            firstCameraX_R0 = lastCameraX_R0;
        }
        maxBoresightDrift = std::max(maxBoresightDrift, angleBetween(boresight_R0, firstBoresight_R0));
        EXPECT_NEAR(angleBetween(boresight_R0, target_R0), offsetAngle, 1e-4F);
    }
    EXPECT_LT(maxBoresightDrift, 1e-4F);
    EXPECT_NEAR(angleBetween(lastCameraX_R0, firstCameraX_R0), (kNumSteps - 1) * kRate * kControlPeriod, 1e-3F);
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// Zero rate: the output attitude does not change across updates.
TEST(MrpRotationTest, ZeroRateHoldsAttitude) {
    const auto cfg = MrpRotationConfig::create(/* sigma_RbR0 = */ Eigen::Vector3f{0.2F, 0.3F, 0.4F},
                                               /* sigma_CnB = */ cameraNominalMounting(),
                                               /* sigma_CB = */ cameraMeasuredMounting(),
                                               /* sigma_RpRc_C = */ Eigen::Vector3f{0.0F, -0.1F, 0.0F},
                                               /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.0F,
                                               /* controlPeriod = */ 1.0F);
    MrpRotationAlgorithm alg{cfg};

    const MrpRotationAttRefInputs attRef{};

    const MrpRotationOutput out0 = alg.update(attRef);
    const MrpRotationOutput out1 = alg.update(attRef);
    const MrpRotationOutput out2 = alg.update(attRef);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out0.sigma_RN(i), out1.sigma_RN(i), tol);
        EXPECT_NEAR(out1.sigma_RN(i), out2.sigma_RN(i), tol);
    }
}

// reInitialize() restarts the spin at identity, so a second pass after reInitialize matches the first.
TEST(MrpRotationTest, ReInitializeRestartsRotationForRepeatability) {
    const auto cfg = MrpRotationConfig::create(/* sigma_RbR0 = */ Eigen::Vector3f{0.1F, -0.2F, 0.3F},
                                               /* sigma_CnB = */ cameraNominalMounting(),
                                               /* sigma_CB = */ cameraMeasuredMounting(),
                                               /* sigma_RpRc_C = */ Eigen::Vector3f{0.0F, 0.0F, 0.05F},
                                               /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                                               /* spinRate = */ 0.01F,
                                               /* controlPeriod = */ 0.5F);
    MrpRotationAlgorithm alg{cfg};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };

    const MrpRotationOutput first0 = alg.update(attRef);
    const MrpRotationOutput first1 = alg.update(attRef);

    alg.reInitialize();
    const MrpRotationOutput second0 = alg.update(attRef);
    const MrpRotationOutput second1 = alg.update(attRef);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(first0.sigma_RN(i), second0.sigma_RN(i), tol);
        EXPECT_NEAR(first1.sigma_RN(i), second1.sigma_RN(i), tol);
    }
}

// After setConfig(cfgB) + reInitialize(), the algorithm behaves exactly like a new algorithm built from cfgB:
// the spin restarts at identity and the step uses cfgB's rotations, rate and controlPeriod.
TEST(MrpRotationTest, ReInitializeAfterSetConfigMatchesFreshAlgorithm) {
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto cfgA = MrpRotationConfig::create(/* sigma_RbR0 = */ Eigen::Vector3f{0.1F, 0.0F, 0.0F},
                                                zero,
                                                zero,
                                                zero,
                                                /* spinAxis_C = */ Eigen::Vector3f::UnitX(),
                                                /* spinRate = */ 0.05F,
                                                /* controlPeriod = */ 0.5F);
    const auto cfgB = MrpRotationConfig::create(/* sigma_RbR0 = */ Eigen::Vector3f{0.0F, 0.5F, 0.0F},
                                                /* sigma_CnB = */ cameraNominalMounting(),
                                                /* sigma_CB = */ cameraMeasuredMounting(),
                                                /* sigma_RpRc_C = */ Eigen::Vector3f{0.0F, 0.0F, 0.1F},
                                                /* spinAxis_C = */ Eigen::Vector3f::UnitY(),
                                                /* spinRate = */ -0.2F,
                                                /* controlPeriod = */ 0.25F);

    MrpRotationAlgorithm alg{cfgA};
    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, -0.1F, 0.0F},
        Eigen::Vector3f::Zero(),
        Eigen::Vector3f::Zero(),
    };
    alg.update(attRef);
    alg.update(attRef);

    alg.setConfig(cfgB);
    alg.reInitialize();

    MrpRotationAlgorithm fresh{cfgB};
    for (int k = 0; k < 3; ++k) {
        const MrpRotationOutput a = alg.update(attRef);
        const MrpRotationOutput b = fresh.update(attRef);
        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(a.sigma_RN), mrpToDcm(b.sigma_RN)), 1e-6F);
    }
}

// setConfig() with a new pointing offset applies it at the next update and keeps the spin phase: an
// algorithm that switches from offset 1 to offset 2 after k steps matches, from then on, an algorithm that
// ran with offset 2 from the start.
TEST(MrpRotationTest, SetConfigAppliesNewOffsetAndKeepsRotationPhase) {
    const Eigen::Vector3f sigma_RbR0{0.2F, -0.1F, 0.3F};
    const Eigen::Vector3f spinAxis_C = Eigen::Vector3f{2.0F, 0.0F, 1.0F}.stableNormalized();
    constexpr float kSpinRate = 0.0223607F;
    constexpr float kControlPeriod = 0.5F;
    const auto cfg1 = MrpRotationConfig::create(sigma_RbR0,
                                                cameraNominalMounting(),
                                                cameraMeasuredMounting(),
                                                /* sigma_RpRc_C = */ Eigen::Vector3f{0.0F, 0.0F, 0.0F},
                                                spinAxis_C,
                                                kSpinRate,
                                                kControlPeriod);
    const auto cfg2 = MrpRotationConfig::create(sigma_RbR0,
                                                cameraNominalMounting(),
                                                cameraMeasuredMounting(),
                                                /* sigma_RpRc_C = */ Eigen::Vector3f{0.0437F, 0.0F, 0.0F},
                                                spinAxis_C,
                                                kSpinRate,
                                                kControlPeriod);

    MrpRotationAlgorithm switched{cfg1};
    MrpRotationAlgorithm reference{cfg2};

    const MrpRotationAttRefInputs attRef{
        Eigen::Vector3f{0.05F, 0.0F, 0.0F},
        Eigen::Vector3f{0.001F, 0.0F, 0.0F},
        Eigen::Vector3f::Zero(),
    };

    for (int k = 0; k < 5; ++k) {
        const MrpRotationOutput a = switched.update(attRef);
        const MrpRotationOutput b = reference.update(attRef);
        EXPECT_GT(dcmMaxAbsDiff(mrpToDcm(a.sigma_RN), mrpToDcm(b.sigma_RN)), 1e-2F) << "offsets differ before switch";
    }

    switched.setConfig(cfg2);

    for (int k = 0; k < 5; ++k) {
        const MrpRotationOutput a = switched.update(attRef);
        const MrpRotationOutput b = reference.update(attRef);
        EXPECT_LT(dcmMaxAbsDiff(mrpToDcm(a.sigma_RN), mrpToDcm(b.sigma_RN)), 1e-6F);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(a.omega_RN_N(i), b.omega_RN_N(i), 1e-6F);
            EXPECT_NEAR(a.domega_RN_N(i), b.domega_RN_N(i), 1e-6F);
        }
    }
}

// The base frame puts body -y on the target (input reference x-axis), the pointing offset tilts the boresight 10 deg
// about body x, and the rate points along the target line in component axes after the offset. With identity mountings
// the component axes are the body axes. The boresight then sweeps a cone around the target: constant 10 deg from it,
// reaching 20 deg from its start after half a turn.
TEST(MrpRotationTest, ConeAroundTargetKeepsConstantHalfAngle) {
    constexpr float halfAngle = 10.0F * kDeg;
    constexpr float rate = 0.02F;
    constexpr float kControlPeriod = 0.5F;
    constexpr int kNumSteps = 2000;  // more than 3 turns

    Eigen::Matrix3f dcm_RbR0;
    dcm_RbR0 << 0.0F, 1.0F, 0.0F, -1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;
    const Eigen::Vector3f sigma_RbR0 = dcmToMrp(dcm_RbR0);
    const Eigen::Vector3f sigma_RpRc_C = std::tan(halfAngle / 4.0F) * Eigen::Vector3f::UnitX();
    const Eigen::Vector3f boresight_B = -Eigen::Vector3f::UnitY();
    const Eigen::Vector3f spinAxis_C = mrpToDcm(sigma_RpRc_C) * boresight_B;
    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();

    const auto cfg = MrpRotationConfig::create(sigma_RbR0, zero, zero, sigma_RpRc_C, spinAxis_C, rate, kControlPeriod);
    MrpRotationAlgorithm alg{cfg};

    const Eigen::Vector3f target_R0 = Eigen::Vector3f::UnitX();
    const MrpRotationAttRefInputs identityRef{};

    Eigen::Vector3f initialBoresight_R0 = Eigen::Vector3f::Zero();
    float minCosFromStart = 1.0F;
    for (int k = 0; k < kNumSteps; ++k) {
        const MrpRotationOutput out = alg.update(identityRef);
        const Eigen::Vector3f boresight_R0 = mrpToDcm(out.sigma_RN).transpose() * boresight_B;
        if (k == 0) {
            initialBoresight_R0 = boresight_R0;
        }
        EXPECT_NEAR(std::acos(std::clamp(boresight_R0.dot(target_R0), -1.0F, 1.0F)), halfAngle, 1e-4F);
        minCosFromStart = std::min(minCosFromStart, boresight_R0.dot(initialBoresight_R0));
    }
    EXPECT_NEAR(minCosFromStart, std::cos(2.0F * halfAngle), 1e-3F);
}

// Direct test of the mrpSwitch path: one forward-Euler step from identity gives sigma_RRp = dt/4 * omega,
// which for this rate exceeds norm 1. The algorithm must land on the shadow-set representative.
TEST(MrpRotationTest, MrpShadowSwitchActivates) {
    const Eigen::Vector3f spinAxis_C = Eigen::Vector3f::UnitX();
    constexpr float kSpinRate = 5.0F;
    constexpr float kControlPeriod = 1.0F;

    const Eigen::Vector3f preSwitchSigma = kControlPeriod * 0.25F * kSpinRate * spinAxis_C;
    ASSERT_GT(preSwitchSigma.stableNorm(), 1.0F) << "Test setup: forward-Euler step must cross norm-1 boundary";
    const Eigen::Vector3f expectedShadow = -preSwitchSigma / preSwitchSigma.squaredNorm();

    const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
    const auto cfg = MrpRotationConfig::create(zero, zero, zero, zero, spinAxis_C, kSpinRate, kControlPeriod);
    MrpRotationAlgorithm alg{cfg};

    // Identity rotations and input reference make sigma_RN equal to the internal (post-switch) sigma_RRp.
    const MrpRotationOutput out = alg.update(MrpRotationAttRefInputs{});

    EXPECT_LT(out.sigma_RN.stableNorm(), 1.0F);
    constexpr float tol = 1e-5F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.sigma_RN(i), expectedShadow(i), tol);
    }
}
