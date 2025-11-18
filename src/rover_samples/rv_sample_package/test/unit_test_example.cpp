#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// Example of a unit test, not ROS specific, but part of a ROS package

#define MAX_ZAZA_LEVEL 10

enum class CatSobreityLevel : int{
    SOBER = 0,
    FEELING_IT = 1,
    HIGH = 3,
    REALLY_HIGH = 6,
    ABSOLUTLEY_ZOOTED = 9,
    ERROR
};
// Keep in reverse order
constexpr CatSobreityLevel ALL_VALID_SOBREITY_LEVELS[] = {
    CatSobreityLevel::SOBER,
    CatSobreityLevel::FEELING_IT,
    CatSobreityLevel::HIGH,
    CatSobreityLevel::REALLY_HIGH,
    CatSobreityLevel::ABSOLUTLEY_ZOOTED
};

class Cat {
public:
    Cat() {};
    void give_catnip(int catnip_kg = 1);

    CatSobreityLevel get_status() const;
    int get_zaza_level() const;
private:

    int zaza_level = 0; 
};

void Cat::give_catnip(int catnip_kg)
{
    this->zaza_level += catnip_kg;
    if(this->zaza_level > MAX_ZAZA_LEVEL)
    {
        this->zaza_level = MAX_ZAZA_LEVEL;
    }
}

CatSobreityLevel Cat::get_status() const
{
    if(this->zaza_level < 0) return CatSobreityLevel::ERROR; // Check if zaza level is valid
    CatSobreityLevel ret = CatSobreityLevel::SOBER;
    for(auto lvl : ALL_VALID_SOBREITY_LEVELS)
    {
        if (this->zaza_level >= static_cast<int>(lvl))
        {
            ret = lvl;
        } 
        else
        {
            break;
        } 
    }
    return ret;
}

int Cat::get_zaza_level() const
{
    return this->zaza_level;
}

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