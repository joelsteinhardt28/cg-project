#pragma once

#include <string>
#include <iostream>


namespace print {
    void info(const std::string& msg);
    void error(const std::string& msg);
    void debug(const std::string& msg);
    void warning(const std::string& msg);
}