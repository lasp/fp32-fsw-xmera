#include "ephemNavConverterTestHelpers.hpp"

// ---------------------------------------------------------------------------
// Property tests
// ---------------------------------------------------------------------------

// Output position exactly equals input position.
TEST(EphemNavConverterTest, PositionPassthrough) {
    propertyPositionPassthrough(100.1, {10060.5, -239847.23, 82148.13}, {-3786.89, 234798.34, -7389.67});
}

// Output velocity exactly equals input velocity.
TEST(EphemNavConverterTest, VelocityPassthrough) {
    propertyVelocityPassthrough(100.1, {10060.5, -239847.23, 82148.13}, {-3786.89, 234798.34, -7389.67});
}

// Output timeTag exactly equals input timeTag.
TEST(EphemNavConverterTest, TimeTagPassthrough) {
    propertyTimeTagPassthrough(100.1, {10060.5, -239847.23, 82148.13}, {-3786.89, 234798.34, -7389.67});
}

// All output components are finite for finite inputs.
TEST(EphemNavConverterTest, OutputIsFinite) {
    propertyOutputIsFinite(100.1, {10060.5, -239847.23, 82148.13}, {-3786.89, 234798.34, -7389.67});
}

// ---------------------------------------------------------------------------
// Edge-case tests
// ---------------------------------------------------------------------------

// All-zero input produces all-zero output.
TEST(EphemNavConverterTest, ZeroInputs) {
    InputEphemerisData input{};

    auto out = EphemNavConverterAlgorithm::update(input);

    EXPECT_EQ(out.timeTag, 0.0);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(out.r_BN_N[i], 0.0);
        EXPECT_EQ(out.v_BN_N[i], 0.0);
    }
}

// Large doubles pass through exactly.
TEST(EphemNavConverterTest, LargeValues) {
    InputEphemerisData input{};
    input.timeTag = 1e200;
    input.r_BdyZero_N = Eigen::Vector3d(1e200, -1e200, 1e150);
    input.v_BdyZero_N = Eigen::Vector3d(-1e200, 1e150, -1e200);

    auto out = EphemNavConverterAlgorithm::update(input);

    EXPECT_EQ(out.timeTag, 1e200);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(out.r_BN_N[i], input.r_BdyZero_N[i]);
        EXPECT_EQ(out.v_BN_N[i], input.v_BdyZero_N[i]);
    }
}

// Negative values pass through exactly.
TEST(EphemNavConverterTest, NegativeValues) {
    InputEphemerisData input{};
    input.timeTag = -42.0;
    input.r_BdyZero_N = Eigen::Vector3d(-1.0, -2.0, -3.0);
    input.v_BdyZero_N = Eigen::Vector3d(-4.0, -5.0, -6.0);

    auto out = EphemNavConverterAlgorithm::update(input);

    EXPECT_EQ(out.timeTag, -42.0);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(out.r_BN_N[i], input.r_BdyZero_N[i]);
        EXPECT_EQ(out.v_BN_N[i], input.v_BdyZero_N[i]);
    }
}

// Subnormal-scale doubles pass through exactly.
TEST(EphemNavConverterTest, SmallValues) {
    InputEphemerisData input{};
    input.timeTag = 1e-300;
    input.r_BdyZero_N = Eigen::Vector3d(1e-300, -1e-300, 5e-310);
    input.v_BdyZero_N = Eigen::Vector3d(-5e-310, 1e-300, -1e-300);

    auto out = EphemNavConverterAlgorithm::update(input);

    EXPECT_EQ(out.timeTag, 1e-300);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(out.r_BN_N[i], input.r_BdyZero_N[i]);
        EXPECT_EQ(out.v_BN_N[i], input.v_BdyZero_N[i]);
    }
}
