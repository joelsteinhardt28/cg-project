#pragma once

#include <vector>
#include <string>

#include <glm/glm.hpp>
#include <pmp/surface_mesh.h>
#include <integer-plane-geometry/geometry.hh>
#include <integer-plane-geometry/point.hh>
#include <integer-plane-geometry/plane.hh>
#include "imgui.h"


// Forward declarations
namespace polyscope {
    class PointCloud;
    class SurfaceMesh;
}

using Point = pmp::vec3;
using Face = std::vector<size_t>;
using Normal = pmp::vec3;

using ExactGeom = ipg::geometry256_x64_n45;
using ExactPoint = ipg::point4<ExactGeom>;
using ExactPlane = ipg::plane<ExactGeom>;

using Color = glm::vec3; // RGB color, each component in [0,1]

const double EPSILON = 1e-6;


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
    bool isSteppingKernel = false;
    int currentPlaneIdx = 0;
    std::vector<Plane> supportPlanes;
    std::vector<ExactPlane> exactSupportPlanes;
    Plane activeCutPlane;
    bool hasActiveCutPlane = false;

    // * Application flags
    bool updateVisuals = true;
    double lastComputeTime = 0.0;
    std::string statusMessage = "";
    ImVec4 statusMessageColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);

    // * AABB Fast Intersection Tracking
    int64_t aabb_min[3] = {0, 0, 0};
    int64_t aabb_max[3] = {0, 0, 0};
    pmp::Vertex aabb_v_min[3];
    pmp::Vertex aabb_v_max[3];
    int skippedCuts = 0;

    // * IO
    std::string targetDir = "./off_files";
    std::vector<std::string> offFiles;
    int selectedOffFileIdx = -1;

};