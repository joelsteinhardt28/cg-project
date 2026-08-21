#pragma once

#include <cmath>
#include "structs.hpp"

/**
 * Reads an OFF 3D model file and extracts points, faces, and vertex/face normals.
 * 
 * @param filename Absolute or relative filesystem path to the .off file
 * @param points Output vector to store vertex positions
 * @param faces Output vector to store face vertex indices
 * @param normals Output vector to store normal vectors
 */
void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces, std::vector<Normal>& normals);

/**
 * Reads an OFF 3D model file and extracts points and faces.
 * 
 * @param filename Absolute or relative filesystem path to the .off file
 * @param points Output vector to store vertex positions
 * @param faces Output vector to store face vertex indices
 */
void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces);
