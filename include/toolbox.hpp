#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>
#include "structs.hpp"


namespace print {
    void info(const std::string& msg);
    void error(const std::string& msg);
    void debug(const std::string& msg);
    void warning(const std::string& msg);
}

struct EuclideanDistance {
    static float measure(Point const& p1, Point const& p2) {
        float dx = p1[0] - p2[0];
        float dy = p1[1] - p2[1];
        float dz = p1[2] - p2[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

namespace geometry {
    std::vector<glm::vec3> clip_polygon_axis(const std::vector<glm::vec3>& poly, int axis, float val, bool isMax);
}