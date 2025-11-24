#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <unit_test_example_class.h>

// Example of a unit test, not ROS specific, but part of a ROS package
// Don't ask about why this class was chosen for the example

TEST_CASE("sample_test_case")
{
    // Setup for every SECTION
    // This code runs fresh before every section
    Cat cat;
    SECTION("Test cat init/construction")
    {
        REQUIRE(cat.get_zaza_level() == 0);
        REQUIRE(cat.get_status() == CatSobreityLevel::SOBER);
    }

    SECTION("Test getting cat high")
    {
        cat.give_catnip();
        REQUIRE(cat.get_zaza_level() == 1);
        REQUIRE(cat.get_status() == CatSobreityLevel::FEELING_IT);

        cat.give_catnip(2);
        REQUIRE(cat.get_zaza_level() == 3);
        REQUIRE(cat.get_status() == CatSobreityLevel::HIGH);

        cat.give_catnip(3);
        REQUIRE(cat.get_zaza_level() == 6);
        REQUIRE(cat.get_status() == CatSobreityLevel::REALLY_HIGH);

        cat.give_catnip(99);
        REQUIRE(cat.get_status() == CatSobreityLevel::ABSOLUTLEY_ZOOTED);
    }

    SECTION("Test max zaza")
    {
        cat.give_catnip(99999);
        REQUIRE(cat.get_zaza_level() == MAX_ZAZA_LEVEL);
    }

    SECTION("Test negative catnip")
    {
        cat.give_catnip(-10);
        REQUIRE(cat.get_status() == CatSobreityLevel::ERROR);
    }
}