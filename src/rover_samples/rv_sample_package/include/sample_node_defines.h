// Once your node needs to define a buncha stuff (more than 5 or 6 defines, but use your judgement) move the defines to a seperate file, <main header name>_defines.h
#define CONTROL_RATE_HZ 60.0
#define YOUR_MOMS_MASS_KG 3000 // Defines should have units in the name.



//Sometimes, you want to define a constant with a type. in a #define, the preprocessor is what replaces the defines with the value.
// For a constant to be type enforced by the compiler, the compiler needs to see it. 
static constexpr bool ON = true;
static constexpr bool OFF = false;


// Now, we should also talk about enums:

// Classic enum we've had since the 90s
// Defaults to int type, but is not scoped by a class. Usage: REVERSE
enum Gears {
    REVERSE,
    NEUTRAL,
    ONE,
    TWO
}; 


enum class GearsScoped { // Now, to use youd need the scope operator. Usage: GearsScoped::REVERSE
    REVERSE,
    NEUTRAL,
    ONE,
    TWO
};

// Can also enforce integer types:
enum class GearsScoped_ : uint8_t { 
    REVERSE = 0,
    NEUTRAL = 1,
    ONE, // would = 2
    TWO
};


// Now, on the border of enums and defines, you can to scoped static constexprs using namespaces:
namespace CarStuff {

    static constexpr char SERIAL_PORT_PATH[] = "/dev/ttyUSB0";
    static constexpr uint8_t NUM_WHEELS = 3;

    // Nested is fine
    namespace Gears {
        static constexpr int REVERSE = -1;
        static constexpr int NEUTRAL = 0;
        static constexpr int ONE = 1;
        static constexpr int TWO = 2;
    }

}