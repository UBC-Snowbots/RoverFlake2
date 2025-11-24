#pragma once
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

