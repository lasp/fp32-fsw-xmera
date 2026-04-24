#include "architecture/testUtilities/eigenFuzzDomains.hpp"
#include "navAggregateTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(NavAggregateAlgorithmFuzz, testNavAggregate)
    .WithDomains(fuzztest::Finite<double>(),
                 xmera::fuzz::Vector3fFinite(),
                 xmera::fuzz::Vector3fFinite(),
                 xmera::fuzz::Vector3fFinite(),
                 fuzztest::Finite<double>(),
                 xmera::fuzz::Vector3dFinite(),
                 xmera::fuzz::Vector3dFinite(),
                 xmera::fuzz::Vector3fFinite(),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG - 1U),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG),
                 fuzztest::InRange(0U, MAX_AGG_NAV_MSG));
