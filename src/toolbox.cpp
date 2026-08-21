#include "toolbox.hpp"
#include "constants.hpp"

#include <iostream>

// Different print functions that print to the terminal with predefined formatting and colors
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


namespace geometry {
    // Clips a polygon along a specified axis at a given value. The polygon is represented as a vector of glm::vec3 points.
    // If isMax is true, the polygon is clipped to keep points with coordinates less than or equal to the value
    std::vector<glm::vec3> clip_polygon_axis(const std::vector<glm::vec3>& polygon, int axis, float val, bool isMax) {
        std::vector<glm::vec3> nextPolygon;
        if (polygon.empty()) return nextPolygon;

        for (size_t i = 0; i < polygon.size(); i++) {
            glm::vec3 p1 = polygon[i];
            glm::vec3 p2 = polygon[(i + 1) % polygon.size()];

            float v1 = p1[axis];
            float v2 = p2[axis];

            auto inside = [&](float v) { return isMax ? (v <= val) : (v >= val); };

            if (inside(v1)) {
                if (inside(v2)) {
                    nextPolygon.push_back(p2);
                } else {
                    float t = (val - v1) / (v2 - v1);
                    nextPolygon.push_back(p1 + t * (p2 - p1));
                }
            } else if (inside(v2)) {
                float t = (val - v1) / (v2 - v1);
                nextPolygon.push_back(p1 + t * (p2 - p1));
                nextPolygon.push_back(p2);
            }
        }
        return nextPolygon;
    }
}