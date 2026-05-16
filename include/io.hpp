#pragma once

#include <string>
#include <vector>
#include <cmath>

#include "structs.hpp"

void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces, std::vector<Normal>& normals);
void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces);

struct EuclideanDistance {
    static float measure(Point const& p1, Point const& p2) {
        float dx = p1[0] - p2[0];
        float dy = p1[1] - p2[1];
        float dz = p1[2] - p2[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};
