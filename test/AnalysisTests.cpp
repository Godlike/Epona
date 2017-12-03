/*
 * Copyright (C) 2017 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#define CATCH_CONFIG_MAIN
#include <Epona/Analysis.hpp>
#include <catch.hpp>
#include <vector>

TEST_CASE("Expected value", "[statistics]")
{
    std::vector<int> values{1, 2, 3};

    REQUIRE(2 == epona::CalculateExpectedValue(values.begin(), values.end()));
}
