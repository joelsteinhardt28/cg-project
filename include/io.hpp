#pragma once

#include <cmath>

#include "structs.hpp"

void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces, std::vector<Normal>& normals);
void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces);
