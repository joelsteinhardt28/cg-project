#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>

#include <pmp/surface_mesh.h>


// Forward declarations
namespace polyscope {
    class PointCloud;
    class SurfaceMesh;
}

using Point = pmp::vec3;
using Face = std::vector<size_t>;
using Normal = pmp::vec3;

inline constexpr int DEFAULT_MAX_LEAF_SIZE = 5;
const float EPSILON = 1e-6f;


struct Plane {
    pmp::vec3 normal;
    float d;  // distance from origin, such that plane equation is normal.x + d = 0

    float distance(const pmp::vec3& p) const {
        return pmp::dot(normal, p) + d;
    }
};


// * Application state struct to hold shared data across the application
struct AppState {
    polyscope::PointCloud* pc = nullptr;
    polyscope::SurfaceMesh* sc = nullptr;
    
    pmp::SurfaceMesh mesh;
    bool meshLoaded = false;

    std::vector<Point> bboxVertices;

    Plane activeCutPlane;
    bool hasActiveCutPlane = false;

    std::string targetDir = "./off_files";
    std::vector<std::string> offFiles;
    int selectedOffFileIdx = -1;
};