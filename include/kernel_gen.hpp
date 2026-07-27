#pragma once

#include <pmp/bounding_box.h>
#include <pmp/algorithms/triangulation.h>
#include <pmp/algorithms/utilities.h>

#include "mesh_utils.hpp"


pmp::SurfaceMesh construct_aabb_mesh(pmp::BoundingBox& bbox);

void init_kernel_stepping(AppState& state);
void step_kernel(AppState& state, bool updateVisuals);
void generate_kernel(AppState& state);
void generate_kernel_parallel(AppState& state);