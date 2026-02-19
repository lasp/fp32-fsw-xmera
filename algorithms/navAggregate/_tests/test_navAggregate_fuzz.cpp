#include "navAggregateTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

FUZZ_TEST(NavAggregateAlgorithmFuzz, testNavAggregate)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::VectorOf(fuzztest::Finite<float>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<float>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<float>()).WithSize(3),
                 fuzztest::Finite<double>(),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<float>()).WithSize(3),
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
