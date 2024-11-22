#include <string>

//Thanks GPT
class ConsoleFormat {
public:
    // Text styles
    static constexpr const char* reset() { return "\033[0m"; }
    static constexpr const char* bold() { return "\033[1m"; }
    static constexpr const char* dim() { return "\033[2m"; }
    static constexpr const char* italic() { return "\033[3m"; } // May not be supported in all terminals.
    static constexpr const char* underline() { return "\033[4m"; }
    static constexpr const char* blink() { return "\033[5m"; }
    static constexpr const char* inverse() { return "\033[7m"; }
    static constexpr const char* hidden() { return "\033[8m"; }
    static constexpr const char* strike() { return "\033[9m"; }

    // Text colors
    static constexpr const char* black() { return "\033[30m"; }
    static constexpr const char* red() { return "\033[31m"; }
    static constexpr const char* green() { return "\033[32m"; }
    static constexpr const char* yellow() { return "\033[33m"; }
    static constexpr const char* blue() { return "\033[34m"; }
    static constexpr const char* magenta() { return "\033[35m"; }
    static constexpr const char* cyan() { return "\033[36m"; }
    static constexpr const char* white() { return "\033[37m"; }

    // Bright text colors
    static constexpr const char* bright_black() { return "\033[90m"; }
    static constexpr const char* bright_red() { return "\033[91m"; }
    static constexpr const char* bright_green() { return "\033[92m"; }
    static constexpr const char* bright_yellow() { return "\033[93m"; }
    static constexpr const char* bright_blue() { return "\033[94m"; }
    static constexpr const char* bright_magenta() { return "\033[95m"; }
    static constexpr const char* bright_cyan() { return "\033[96m"; }
    static constexpr const char* bright_white() { return "\033[97m"; }

    // Background colors
    static constexpr const char* bg_black() { return "\033[40m"; }
    static constexpr const char* bg_red() { return "\033[41m"; }
    static constexpr const char* bg_green() { return "\033[42m"; }
    static constexpr const char* bg_yellow() { return "\033[43m"; }
    static constexpr const char* bg_blue() { return "\033[44m"; }
    static constexpr const char* bg_magenta() { return "\033[45m"; }
    static constexpr const char* bg_cyan() { return "\033[46m"; }
    static constexpr const char* bg_white() { return "\033[47m"; }

    // Bright background colors
    static constexpr const char* bg_bright_black() { return "\033[100m"; }
    static constexpr const char* bg_bright_red() { return "\033[101m"; }
    static constexpr const char* bg_bright_green() { return "\033[102m"; }
    static constexpr const char* bg_bright_yellow() { return "\033[103m"; }
    static constexpr const char* bg_bright_blue() { return "\033[104m"; }
    static constexpr const char* bg_bright_magenta() { return "\033[105m"; }
    static constexpr const char* bg_bright_cyan() { return "\033[106m"; }
    static constexpr const char* bg_bright_white() { return "\033[107m"; }
};
