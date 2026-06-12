#include "kernel_gen.hpp"


// * Given a bounding box, construct a surface mesh representing its axis-aligned bounding box (AABB).
pmp::SurfaceMesh construct_aabb_mesh(pmp::BoundingBox& bbox) {
    pmp::SurfaceMesh aabb;

    pmp::Point min = bbox.min();
    pmp::Point max = bbox.max();

    // Construct the eight vertices of the AABB
    auto v0 = aabb.add_vertex(pmp::Point(min[0], min[1], min[2]));
    auto v1 = aabb.add_vertex(pmp::Point(max[0], min[1], min[2]));
    auto v2 = aabb.add_vertex(pmp::Point(max[0], max[1], min[2]));
    auto v3 = aabb.add_vertex(pmp::Point(min[0], max[1], min[2]));
    auto v4 = aabb.add_vertex(pmp::Point(min[0], min[1], max[2]));
    auto v5 = aabb.add_vertex(pmp::Point(max[0], min[1], max[2]));
    auto v6 = aabb.add_vertex(pmp::Point(max[0], max[1], max[2]));
    auto v7 = aabb.add_vertex(pmp::Point(min[0], max[1], max[2]));

    // Construct the six faces
    aabb.add_face({v0, v3, v2, v1}); // Bottom face
    aabb.add_face({v4, v5, v6, v7}); // Top face
    aabb.add_face({v0, v1, v5, v4}); // Front face
    aabb.add_face({v1, v2, v6, v5}); // Right face
    aabb.add_face({v2, v3, v7, v6}); // Back face
    aabb.add_face({v3, v0, v4, v7}); // Left face

    for (auto face : aabb.faces()) {
        triangulate(aabb, face);
    }
    
    return aabb;
}

void init_kernel_stepping(AppState& state) {
    if (!state.meshLoaded || state.mesh.is_empty()) return;

    print::info("Initializing kernel stepping...");

    // * Genus early termination
    int euler = state.mesh.n_vertices() - state.mesh.n_edges() + state.mesh.n_faces();
    if (euler < 2) {
        polyscope::info("Mesh has genus > 0. Kernel is empty.");
        return;
    }

    // Initialize intermediate kernel with AABB
    pmp::BoundingBox bbox = pmp::bounds(state.mesh);
    state.kHat = construct_aabb_mesh(bbox);

    // Identify concave faces
    std::vector<bool> isConcaveFace = mesh_utils::identify_concave_faces(state.mesh);

    // * Concavity early termination
    // if (std::none_of(isConcaveFace.begin(), isConcaveFace.end(), [](bool v) { return v; })) {
    //     polyscope::info("Mesh has no concave faces. Kernel is the mesh itself.");
    //     state.kHat = state.mesh;
    //     return;
    // }

    std::vector<Plane> concavePlanes;
    std::vector<Plane> convexPlanes;

    // Extract supporting planes from input mesh
    state.supportPlanes.clear();
    state.supportPlanes.reserve(state.mesh.n_faces());

    // * Iterate over faces to compute their supporting planes
    for (auto face : state.mesh.faces()) {
        auto it = state.mesh.vertices(face).begin();
        pmp::Point vA = state.mesh.position(*it); ++it;
        pmp::Point vB = state.mesh.position(*it); ++it;
        pmp::Point vC = state.mesh.position(*it);

        // Compute the normal of the face
        pmp::vec3 e1 = vB - vA;
        pmp::vec3 e2 = vC - vA;
        pmp::vec3 normal = pmp::cross(e1, e2);

        if (pmp::norm(normal) > EPSILON) {      // Check for degenerate face
            normal = pmp::normalize(normal);
            float d = -pmp::dot(normal, vA);    // Plane offset d_i = -n_i^\top \cdot v_a

            // Separate concave vs convex faces based on the precomputed list
            bool isConcave = face.idx() < static_cast<unsigned int>(isConcaveFace.size()) && isConcaveFace[face.idx()];
            std::vector<Plane>& targetList = isConcave ? concavePlanes : convexPlanes;
            targetList.push_back({normal, d});
        }
    }

    // Prioritize concave planes first, then convex planes
    state.supportPlanes.insert(state.supportPlanes.end(), concavePlanes.begin(), concavePlanes.end());
    state.supportPlanes.insert(state.supportPlanes.end(), convexPlanes.begin(), convexPlanes.end());

    state.currentPlaneIdx = 0;
    state.isSteppingKernel = true;

    // Register initial kernel
    if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
    state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
    state.kSMesh->setSurfaceColor(constants::colors::kernel);
    state.kSMesh->setTransparency(constants::transparencies::kernel);
}

void step_kernel(AppState& state) {
    if (!state.isSteppingKernel || state.supportPlanes.empty()) return;
    print::debug("Stepping kernel: processing plane idx " + std::to_string(state.currentPlaneIdx));

    if (static_cast<size_t>(state.currentPlaneIdx) >= state.supportPlanes.size()) {
        print::info("All planes processed.");
        state.isSteppingKernel = false;
        return;
    }

    if (state.kHat.is_empty()) {
        print::info("Kernel is empty.");
        state.isSteppingKernel = false;
        return;
    }

    Plane& plane = state.supportPlanes[state.currentPlaneIdx];

    // TODO: Remove this
    plane.d += EPSILON * 5.0f;  // add a small offset to ensure we don't run into numerical issues with coplanar faces
    
    // Visualize the current support plane
    mesh_utils::visualize_cut_plane(state, plane);

    // Perform the cut
    mesh_utils::cut_at_plane_linear_search(state, state.kHat, plane);
    
    state.currentPlaneIdx++;

    // Update the kernel surface mesh in Polyscope
    if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
    state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
    state.kSMesh->setSurfaceColor(constants::colors::kernel);
    state.kSMesh->setTransparency(constants::transparencies::kernel);

    // if (static_cast<size_t>(state.currentPlaneIdx) >= state.supportPlanes.size()) {
    //     print::info("Kernel generation completed.");
    //     state.isSteppingKernel = false;
    // }
}

void generate_kernel(AppState& state) {
    init_kernel_stepping(state);
    while (state.isSteppingKernel) {
        step_kernel(state);
    }
}