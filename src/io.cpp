#include "io.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

#include "polyscope/messages.h"

/*
 * Reads an OFF file and populates the provided vectors with points, faces, and optionally normals.
 */
void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces, std::vector<Normal>& normals) {

    points.clear();
    faces.clear();
    normals.clear();

    std::ifstream file(filename);
    if (!file.is_open()) {
        polyscope::error("Could not open file: " + filename);
        return;
    }

    // Check for a valid OFF header
    std::string header;
    file >> header;
    if (header.substr(0, 3) != "OFF") {
        polyscope::error("Invalid OFF file: Missing OFF header.");
        return;
    }

    std::size_t nVertices, nFaces, nEdges;
    if (header.length() > 3) {
        std::istringstream iss(header.substr(3));
        if (!(iss >> nVertices)) {
            file >> nVertices;
        }
    } else {
        file >> nVertices;
    }
    file >> nFaces >> nEdges;

    // Read in the next nVertices lines for vertices
    points.resize(nVertices);
    for (std::size_t i = 0; i < nVertices; ++i) {
        file >> points[i][0] >> points[i][1] >> points[i][2];
    }

    // Read in the next nFaces lines for faces
    faces.resize(nFaces);
    for (std::size_t i = 0; i < nFaces; ++i) {
        std::size_t nVerticesInFace;
        file >> nVerticesInFace;
        faces[i].resize(nVerticesInFace);
        for (std::size_t j = 0; j < nVerticesInFace; ++j) {
            file >> faces[i][j];
        }
    }

    polyscope::info("Successfully read " + std::to_string(nVertices) + " vertices and " + std::to_string(nFaces) + " faces from OFF file.");
    file.close();
}

void readOff(const std::string& filename, std::vector<Point>& points, std::vector<Face>& faces) {
    std::vector<Normal> dummy_normals;
    readOff(filename, points, faces, dummy_normals);  // reuse the full version
}
