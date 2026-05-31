#pragma once

#include "structs.hpp"
#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

namespace mesh_utils {
    polyscope::SurfaceMesh* registerPmpMesh(const std::string& name, const pmp::SurfaceMesh& mesh);
    polyscope::PointCloud* registerPmpPointCloud(const std::string& name, const pmp::SurfaceMesh& mesh);
    void registerBoundingBox(AppState& state);
    void generate_random_bbox_plane(AppState& state);
    void cut_at_plane(AppState& state, const Plane& plane);
}
