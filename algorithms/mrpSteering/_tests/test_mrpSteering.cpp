#include "mrpSteeringTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(MrpSteeringTest, ReferenceTest) { testMrpSteering(std::vector<float>{0.4, 0.1, -0.3}, 1.0, 2.0, 3.0, false); }

TEST(MrpSteeringTest, ConfigConstruction) { testMrpSteeringConfigConstruction(); }

TEST(MrpSteeringTest, ConfigValues) { testMrpSteeringConfigValues(); }

TEST(MrpSteeringTest, ConfigValidators) { testMrpSteeringConfigValidators(); }

TEST(MrpSteeringTest, ConfigSetters) { testMrpSteeringConfigSetters(); }

TEST(MrpSteeringTest, AlgorithmConstruction) { testMrpSteeringAlgorithmConstruction(); }
