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
    const NavAggregateAttSelection validAtt{};
    const NavAggregateTransSelection validTrans{};

    // --- Valid baseline does not throw ---
    EXPECT_NO_THROW(NavAggregateConfig::create(validAtt, validTrans));

    // --- An attitude selection index equal to or greater than the max message count throws ---
    for (uint32_t NavAggregateAttSelection::* idx : {&NavAggregateAttSelection::attTimeIdx,
                                                     &NavAggregateAttSelection::attIdx,
                                                     &NavAggregateAttSelection::rateIdx,
                                                     &NavAggregateAttSelection::sunIdx}) {
        NavAggregateAttSelection att{};
        att.*idx = MAX_AGG_NAV_MSG;
        EXPECT_THROW(NavAggregateConfig::create(att, validTrans), fsw::invalid_argument);
    }

    // --- A translation selection index equal to or greater than the max message count throws ---
    for (uint32_t NavAggregateTransSelection::* idx : {&NavAggregateTransSelection::transTimeIdx,
                                                       &NavAggregateTransSelection::posIdx,
                                                       &NavAggregateTransSelection::velIdx,
                                                       &NavAggregateTransSelection::dvIdx}) {
        NavAggregateTransSelection trans{};
        trans.*idx = MAX_AGG_NAV_MSG;
        EXPECT_THROW(NavAggregateConfig::create(validAtt, trans), fsw::invalid_argument);
    }

    // --- A message count greater than the max message count throws ---
    {
        NavAggregateAttSelection att{};
        att.attMsgCount = MAX_AGG_NAV_MSG + 1U;
        EXPECT_THROW(NavAggregateConfig::create(att, validTrans), fsw::invalid_argument);
    }
    {
        NavAggregateTransSelection trans{};
        trans.transMsgCount = MAX_AGG_NAV_MSG + 1U;
        EXPECT_THROW(NavAggregateConfig::create(validAtt, trans), fsw::invalid_argument);
    }
}
