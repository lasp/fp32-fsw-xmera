#include "ephemNavConverterTestHelpers.hpp"
#include <fuzztest/fuzztest.h>

// ---------------------------------------------------------------------------
// Property fuzz tests
// ---------------------------------------------------------------------------

FUZZ_TEST(EphemNavConverterPropertyFuzz, propertyPositionPassthrough)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3));

FUZZ_TEST(EphemNavConverterPropertyFuzz, propertyVelocityPassthrough)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3));

FUZZ_TEST(EphemNavConverterPropertyFuzz, propertyTimeTagPassthrough)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3));

FUZZ_TEST(EphemNavConverterPropertyFuzz, propertyOutputIsFinite)
    .WithDomains(fuzztest::Finite<double>(),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3),
                 fuzztest::VectorOf(fuzztest::Finite<double>()).WithSize(3));
