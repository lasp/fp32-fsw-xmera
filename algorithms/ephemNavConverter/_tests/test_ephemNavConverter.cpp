#include "ephemNavConverterTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(EphemNavConverterTest, ReferenceTest) {
    testEphemNavConverter(
        100.1, std::vector<double>{10060.5, -239847.23, 82148.13}, std::vector<double>{-3786.89, 234798.34, -7389.67});
}
