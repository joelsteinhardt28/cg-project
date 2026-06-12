#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>

#include <glm/glm.hpp>
#include <pmp/surface_mesh.h>


// Forward declarations
namespace polyscope {
    class PointCloud;
    class SurfaceMesh;
}

using Point = pmp::vec3;
using Face = std::vector<size_t>;
using Normal = pmp::vec3;

using Color = glm::vec3; // RGB color, each component in [0,1]

const float EPSILON = 1e-6f;


struct Plane {
    pmp::vec3 normal;
    float d;  // distance from origin, such that plane equation is normal.x + d = 0

    // Compute the distance from a point to the plane
    float distance(const pmp::vec3& p) const {
        return pmp::dot(normal, p) + d;
    }
};


// * Application state struct to hold shared data across the application
struct AppState {
    polyscope::PointCloud* pc = nullptr;
    polyscope::SurfaceMesh* oSMesh = nullptr;    // The surface mesh of the original mesh
    polyscope::SurfaceMesh* kSMesh = nullptr;    // The surface mesh of the kernel

    pmp::SurfaceMesh mesh;  // The original mesh
    pmp::SurfaceMesh kHat;  // Intermediate mesh kernel, init as AABB of mesh
    bool meshLoaded = false;

    std::vector<Point> bboxVertices;

    Plane activeCutPlane;
    bool hasActiveCutPlane = false;

    // Kernel Stepping State
    bool isSteppingKernel = false;
    int currentPlaneIdx = 0;
    std::vector<Plane> supportPlanes;

    std::string targetDir = "./off_files";
    std::vector<std::string> offFiles;
    int selectedOffFileIdx = -1;
};