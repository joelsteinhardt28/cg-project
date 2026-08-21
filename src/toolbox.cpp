#include "toolbox.hpp"
#include "constants.hpp"

#include <iostream>

namespace print {
    void info(const std::string& msg) {
        if (globalSettings::showAnyLogs) {
            std::cout << "\033[1;34m[INFO] \033[0;34m" << msg << "\033[0m" << std::endl;
        }
    }

    void error(const std::string& msg) {
        if (globalSettings::showAnyLogs) {
            std::cerr << "\033[1;31m[ERROR] \033[0;31m" << msg << "\033[0m" << std::endl;
        }
    }

    void debug(const std::string& msg) {
        if (globalSettings::showAnyLogs && globalSettings::showDebugLogs) {
            std::cout << "\033[1;36m[DEBUG] \033[0;36m" << msg << "\033[0m" << std::endl;
        }
    }

    void warning(const std::string& msg) {
        if (globalSettings::showAnyLogs) {
            std::cout << "\033[1;33m[WARNING] \033[0;33m" << msg << "\033[0m" << std::endl;
        }
    }
}