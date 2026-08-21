#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include <pmp/algorithms/triangulation.h>

#include "constants.hpp"
#include "toolbox.hpp"

namespace mesh_utils {

    // Helper functions
    polyscope::SurfaceMesh* register_pmp_mesh(const std::string& name, const pmp::SurfaceMesh& mesh);
    polyscope::PointCloud* register_pmp_pc(const std::string& name, const pmp::SurfaceMesh& mesh);
    void register_bbox(AppState& state);
    std::pair<Point, Point> compute_bbox_min_max(const std::vector<Point>& bboxVertices);
    void visualize_cut_plane(AppState& state, const Plane& plane);
    void generate_random_bbox_plane(AppState& state);
    std::vector<bool> identify_concave_faces(const pmp::SurfaceMesh& mesh);
    void visualize_face_normals(AppState& state);

    // Plane-Mesh Cutting
    pmp::Halfedge edge_descent(pmp::SurfaceMesh& mesh, const Plane& plane, const AppState* state = nullptr);
    pmp::Halfedge edge_descent_exact(pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane, const AppState* state = nullptr);
    void cut_at_plane_exact(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane, bool updateVisuals);

    size_t get_linear_fallback_count();
    void reset_linear_fallback_count();
}
