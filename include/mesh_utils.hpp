#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include <pmp/algorithms/triangulation.h>

#include "constants.hpp"
#include "toolbox.hpp"

namespace mesh_utils {

    /**
     * Registers or updates a PMP surface mesh in Polyscope.
     * 
     * @param name Unique identifier name for the structure in Polyscope
     * @param mesh The PMP surface mesh to register
     * @return `polyscope::SurfaceMesh*` pointer to the registered Polyscope surface mesh
     */
    polyscope::SurfaceMesh* register_pmp_mesh(const std::string& name, const pmp::SurfaceMesh& mesh);

    /**
     * Registers or updates a PMP point cloud representation of mesh vertices in Polyscope.
     * 
     * @param name Unique identifier name for the structure in Polyscope
     * @param mesh The PMP surface mesh whose vertices to extract
     * @return `polyscope::PointCloud*` Pointer to the registered Polyscope point cloud
     */
    polyscope::PointCloud* register_pmp_pc(const std::string& name, const pmp::SurfaceMesh& mesh);

    /**
     * Computes and registers the bounding box of the active mesh in Polyscope as a curve network.
     * 
     * @param state The global application state
     */
    void register_bbox(AppState& state);

    /**
     * Computes the minimum and maximum coordinate bounds (minP, maxP) of a bounding box vertex collection.
     * 
     * @param bboxVertices List of bounding box 3D corner vertices
     * @return std::pair<Point, Point> Pair containing minP and maxP corner points
     */
    std::pair<Point, Point> compute_bbox_min_max(const std::vector<Point>& bboxVertices);

    /**
     * Visualizes a cutting plane as a bounded quad clipped against the mesh bounding box,
     * along with its outward normal vector curve network.
     * 
     * @param state The global application state
     * @param plane The 3D plane to visualize
     */
    void visualize_cut_plane(AppState& state, const Plane& plane);

    /**
     * Generates a random cutting plane intersecting the bounding box of the loaded mesh and visualizes it.
     * 
     * @param state The global application state
     */
    void generate_random_bbox_plane(AppState& state);

    /**
     * Evaluates face concavity across mesh edges by calculating signed tetrahedra volumes.
     * 
     * @param mesh The PMP surface mesh to evaluate
     * @return std::vector<bool> Vector where entry i is true if face i is concave
     */
    std::vector<bool> identify_concave_faces(const pmp::SurfaceMesh& mesh);

    /**
     * Computes face normal vectors of the active mesh and visualizes them in Polyscope.
     * 
     * @param state The global application state
     */
    void visualize_face_normals(AppState& state);

    /**
     * Performs greedy edge descent using floating-point distances to find a halfedge crossing the plane.
     * 
     * @param mesh The PMP surface mesh
     * @param plane The floating-point target plane
     * @param state Optional pointer to AppState for multi-start AABB seed initialization
     * @return pmp::Halfedge The crossing halfedge if found, or an invalid halfedge
     */
    pmp::Halfedge edge_descent(pmp::SurfaceMesh& mesh, const Plane& plane, const AppState* state = nullptr);

    /**
     * Performs exact multi-start edge descent using 256-bit IPG geometric predicates.
     * Falls back to linear edge search if greedy descent gets trapped in local extrema.
     * 
     * @param mesh The PMP surface mesh
     * @param plane Floating-point target plane
     * @param exactPlane Exact 256-bit integer plane definition
     * @param state Optional pointer to AppState for multi-start AABB seed initialization
     * @return pmp::Halfedge The crossing halfedge if found, or an invalid halfedge
     */
    pmp::Halfedge edge_descent_exact(pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane, const AppState* state = nullptr);

    /**
     * Cuts a convex PMP mesh at the specified plane, discarding the portion in the positive half-space
     * and constructing an exact planar cap face to seal the resulting hole.
     * 
     * @param state The global application state
     * @param mesh The intermediate kernel mesh to cut
     * @param plane Floating-point target plane
     * @param exactPlane Exact 256-bit integer plane definition
     * @param updateVisuals Whether to update Polyscope 3D visualization after the cut
     */
    void cut_at_plane_exact(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane, bool updateVisuals);

    /**
     * Gets the current cumulative count of linear fallbacks triggered during edge descent.
     * 
     * @return size_t Total fallback invocation count
     */
    size_t get_linear_fallback_count();

    /**
     * Resets the cumulative linear fallback counter to zero.
     */
    void reset_linear_fallback_count();
}
