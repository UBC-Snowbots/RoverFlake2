#include <unit_test_example_class.h>

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