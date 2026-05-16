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

using Point = std::array<float, 3>;
using Face = std::vector<size_t>;
using Normal = std::array<float, 3>;

inline constexpr int DEFAULT_MAX_LEAF_SIZE = 5;


// * Application state struct to hold shared data across the application
struct AppState {
    polyscope::PointCloud* pc = nullptr;
    polyscope::SurfaceMesh* sc = nullptr;
    
    pmp::SurfaceMesh mesh;
    bool meshLoaded = false;

    std::vector<Point> bboxVertices;

    std::string targetDir = "./off_files";
    std::vector<std::string> offFiles;
    int selectedOffFileIdx = -1;
};