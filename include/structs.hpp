#pragma once

#include <vector>
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


/**
 * Represents a plane in 3D space, defined implicitly by normal.x + d = 0, where normal is the plane normal
 * and d is the distance offset from the origin.
 */
struct Plane {
    pmp::vec3 normal;
    float d;

    /**
     * Compute the distance from a point to the plane.
     */
    float distance(const pmp::vec3& p) const {
        return pmp::dot(normal, p) + d;
    }

    /**
     * Returns if the point is on the positive side of the plane.
     */
    bool is_point_on_positive_side(const pmp::vec3& p) const {
        return distance(p) > EPSILON;
    }
};


enum class CutAlgorithm {
    Standard,
    LinearSearch
};


/**
 *  Application state struct to hold shared data across the application
 */
struct AppState {

    // * Polyscope structures
    polyscope::PointCloud* pc = nullptr;
    polyscope::SurfaceMesh* oSMesh = nullptr;    // The surface mesh of the original mesh
    polyscope::SurfaceMesh* kSMesh = nullptr;    // The surface mesh of the kernel

    // * PMP structures
    pmp::SurfaceMesh mesh;  // The original mesh
    pmp::SurfaceMesh kHat;  // Intermediate mesh kernel, init as AABB of mesh
    bool meshLoaded = false;

    std::vector<Point> bboxVertices;

    // * Kernel generation state
    CutAlgorithm selectedCutAlgorithm = CutAlgorithm::Standard;
    bool isSteppingKernel = false;
    int currentPlaneIdx = 0;
    std::vector<Plane> supportPlanes;
    Plane activeCutPlane;
    bool hasActiveCutPlane = false;

    // * Application flags
    bool updateVisuals = false;

    // * IO
    std::string targetDir = "./off_files";
    std::vector<std::string> offFiles;
    int selectedOffFileIdx = -1;

};