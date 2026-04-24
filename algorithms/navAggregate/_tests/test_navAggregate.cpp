#include "navAggregateTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(NavAggregateTest, ReferenceTest) {
    testNavAggregate(7489.3,
                     Eigen::Vector3f{234.5, -243.23, 134.13},
                     Eigen::Vector3f{-867.89, 342.34, -876.67},
                     Eigen::Vector3f{4.89, 6.12, 423.48},
                     100.1,
                     Eigen::Vector3d{10060.5, -239847.23, 82148.13},
                     Eigen::Vector3d{-3786.89, 234798.34, -7389.67},
                     Eigen::Vector3f{2345.89, 3432.12, 2342.48},
                     1,
                     2,
                     3,
                     4,
                     5,
                     6,
                     7,
                     8,
                     10,
                     10);
}

TEST(NavAggregateTest, SetupTest) {
    NavAggregateAlgorithm alg{};

    // --- Test expected exceptions ---

    // MsgIdx equal to or greater than max message count
    EXPECT_THROW(alg.setAttTimeIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    EXPECT_THROW(alg.setTransTimeIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    EXPECT_THROW(alg.setAttIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    EXPECT_THROW(alg.setRateIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    EXPECT_THROW(alg.setPosIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    EXPECT_THROW(alg.setVelIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    EXPECT_THROW(alg.setDvIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    EXPECT_THROW(alg.setSunIdx(MAX_AGG_NAV_MSG), fs::invalid_argument);
    // MsgCount greater than max message count
    EXPECT_THROW(alg.setAttMsgCount(MAX_AGG_NAV_MSG + 1), fs::invalid_argument);
    EXPECT_THROW(alg.setTransMsgCount(MAX_AGG_NAV_MSG + 1), fs::invalid_argument);
}
