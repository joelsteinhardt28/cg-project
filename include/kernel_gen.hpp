#pragma once

#include <pmp/bounding_box.h>
#include <pmp/algorithms/triangulation.h>
#include <pmp/algorithms/utilities.h>
#include <iostream>

#include "structs.hpp"
#include "mesh_utils.hpp"
#include "toolbox.hpp"
#include "constants.hpp"


pmp::SurfaceMesh construct_aabb_mesh(pmp::BoundingBox& bbox);
void generate_kernel(AppState& state);
void init_kernel_stepping(AppState& state);
void step_kernel(AppState& state, bool updateVisuals);