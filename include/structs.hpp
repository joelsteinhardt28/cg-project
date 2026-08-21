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
 * Visual rendering and Polyscope integration state.
 */
struct VisualState {
    polyscope::PointCloud* pc = nullptr;
    polyscope::SurfaceMesh* oSMesh = nullptr;    ///< Registered original mesh in Polyscope
    polyscope::SurfaceMesh* kSMesh = nullptr;    ///< Registered kernel mesh in Polyscope
    bool updateVisuals = true;                   ///< Whether to update 3D rendering during stepping
    std::string statusMessage = "";              ///< Current in-app status message
    ImVec4 statusMessageColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); ///< Text color for status message
};

/**
 * Bounding box and cut tracking metrics.
 */
struct TrackingState {
    int64_t aabb_min[3] = {0, 0, 0};  ///< Exact integer AABB minimum bounds
    int64_t aabb_max[3] = {0, 0, 0};  ///< Exact integer AABB maximum bounds
    pmp::Vertex aabb_v_min[3];        ///< Extreme minimum vertices for multi-start edge descent
    pmp::Vertex aabb_v_max[3];        ///< Extreme maximum vertices for multi-start edge descent
    int skippedCuts = 0;              ///< Number of redundant plane cuts skipped
};

/**
 * Parallelization partitioning strategies for multi-threaded OpenMP kernel generation.
 */
enum class ParallelStrategy {
    SpatialOctants,    ///< Partition faces by centroid location in 8 spatial bounding octants
    SimilarNormals,    ///< Group faces with similar outward normal orientations
    DissimilarNormals  ///< Distribute face planes with similar normals evenly across different threads
};

/**
 * Kernel computation state and plane queues.
 */
struct KernelState {
    pmp::SurfaceMesh kHat;                 ///< Intermediate kernel mesh (initialized as mesh AABB)
    bool isSteppingKernel = false;         ///< True if manual stepping mode is active
    int currentPlaneIdx = 0;               ///< Index of next cutting plane in queue
    std::vector<Plane> supportPlanes;      ///< Queue of floating-point supporting planes
    std::vector<ExactPlane> exactSupportPlanes; ///< Queue of exact 256-bit supporting planes
    Plane activeCutPlane;                  ///< Active cutting plane currently being processed
    bool hasActiveCutPlane = false;        ///< True if activeCutPlane is set and visible
    double lastComputeTime = 0.0;          ///< Execution runtime in seconds of last computation
    ParallelStrategy parallelStrategy = ParallelStrategy::SpatialOctants; ///< Selected strategy
};

/**
 * File I/O state for loading .off models.
 */
struct IOState {
    std::string targetDir = "./off_files";      ///< Directory path containing .off files
    std::vector<std::string> offFiles;          ///< List of discovered .off file names
    int selectedOffFileIdx = -1;                ///< Index of currently loaded file
};

/**
 * Top-level application state container holding core mesh data and subsystem sub-structs.
 */
struct AppState {
    // * Core Mesh Data
    pmp::SurfaceMesh mesh;               ///< The active original surface mesh
    bool meshLoaded = false;             ///< True if a valid mesh is currently loaded
    std::vector<Point> bboxVertices;     ///< Corner vertices of the mesh bounding box

    // * Sub-structs
    VisualState visuals;
    TrackingState tracking;
    KernelState kernel;
    IOState io;
};