#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>
#include "structs.hpp"

/**
 * Console logging helpers with color output formatting.
 */
namespace print {
    void info(const std::string& msg);
    void error(const std::string& msg);
    void debug(const std::string& msg);
    void warning(const std::string& msg);
}

/**
 * Utility struct for measuring 3D Euclidean distances between points.
 */
struct EuclideanDistance {
    static float measure(Point const& p1, Point const& p2) {
        float dx = p1[0] - p2[0];
        float dy = p1[1] - p2[1];
        float dz = p1[2] - p2[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

/**
 * General 3D computational geometry utilities.
 */
namespace geometry {
    /**
     * Clips a 3D convex polygon against an axis-aligned bounding plane (Sutherland-Hodgman clipping on 1 axis).
     * 
     * @param poly Input 3D polygon vertex sequence
     * @param axis Coordinate axis index (0=X, 1=Y, 2=Z)
     * @param val Coordinate threshold value of the clipping plane
     * @param isMax True to clip upper space (<= val), false to clip lower space (>= val)
     * @return std::vector<glm::vec3> The clipped 3D polygon vertex sequence
     */
    std::vector<glm::vec3> clip_polygon_axis(const std::vector<glm::vec3>& poly, int axis, float val, bool isMax);
}