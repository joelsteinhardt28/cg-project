#pragma once

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include <pmp/algorithms/triangulation.h>

#include "structs.hpp"
#include "toolbox.hpp"
#include "constants.hpp"

namespace mesh_utils {
    polyscope::SurfaceMesh* register_pmp_mesh(const std::string& name, const pmp::SurfaceMesh& mesh);
    polyscope::PointCloud* register_pmp_pc(const std::string& name, const pmp::SurfaceMesh& mesh);
    void register_bbox(AppState& state);
    void visualize_cut_plane(AppState& state, const Plane& plane);
    void generate_random_bbox_plane(AppState& state);

    void cut_at_plane(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, bool updateVisuals);
    void cut_at_plane_linear_search(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, bool updateVisuals);

    std::vector<bool> identify_concave_faces(const pmp::SurfaceMesh& mesh);
}
