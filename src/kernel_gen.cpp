#include "kernel_gen.hpp"


namespace {
    struct UnionFind {
        std::vector<int> parent;
        std::vector<int> rank;

        UnionFind(int n) : parent(n), rank(n, 0) {
            for (int i = 0; i < n; ++i) parent[i] = i;
        }

        // Looks up which group face i belong to, with path compression
        int find(int i) {
            if (parent[i] == i) return i;
            return parent[i] = find(parent[i]);
        }

        // Merge two groups together, with union by rank
        void unite(int i, int j) {  
            int root_i = find(i);
            int root_j = find(j);
            if (root_i != root_j) {
                if (rank[root_i] > rank[root_j]) {
                    parent[root_j] = root_i;
                } else if (rank[root_i] < rank[root_j]) {
                    parent[root_i] = root_j;
                } else {
                    parent[root_j] = root_i;
                    rank[root_i]++;
                }
            }
        }
    };
}


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


/**
 * Intiializes the kernel stepping process by first checking for early termination conditions, then performing
 * plane clustering to group coplanar faces and sort them by concavity, and finally registering the 
 * intermediate kernel as the AABB of the original mesh.
 */
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
    if (std::none_of(isConcaveFace.begin(), isConcaveFace.end(), [](bool v) { return v; })) {
        polyscope::info("Mesh has no concave faces. Kernel is the mesh itself.");
        state.kHat = state.mesh;
        return;
    }

    // * Plane clustering and concavity prioritization
    // 1. Precompute all valid face planes
    size_t numFaces = state.mesh.n_faces();
    std::vector<Plane> facePlanes(numFaces);
    std::vector<bool> validPlane(numFaces, false);

    for (auto face : state.mesh.faces()) {
        auto it = state.mesh.vertices(face).begin();
        pmp::Point vA = state.mesh.position(*it); ++it;
        pmp::Point vB = state.mesh.position(*it); ++it;
        pmp::Point vC = state.mesh.position(*it);

        pmp::vec3 normal = pmp::cross(vB - vA, vC - vA);
        if (pmp::norm(normal) > EPSILON) {      // Check for degenerate face
            normal = pmp::normalize(normal);
            float d = -pmp::dot(normal, vA);
            facePlanes[face.idx()] = {normal, d};
            validPlane[face.idx()] = true;
        }
    }

    // 2. Group connected coplanar faces using Union-Find
    UnionFind uf(numFaces);
    const float COPLANAR_NORMAL_TOLERANCE = 1.0f - 1e-5f; // approx 0.25 degrees
    const float COPLANAR_DIST_TOLERANCE = 1e-4f;

    for (auto edge : state.mesh.edges()) {
        if (state.mesh.is_boundary(edge)) continue;

        // Acquire the two faces adjacent to this edge via the two halfedges
        pmp::Face f0 = state.mesh.face(state.mesh.halfedge(edge, 0));
        pmp::Face f1 = state.mesh.face(state.mesh.halfedge(edge, 1));

        if (validPlane[f0.idx()] && validPlane[f1.idx()]) {
            const Plane& p0 = facePlanes[f0.idx()];
            const Plane& p1 = facePlanes[f1.idx()];

            // Check if these adjacent faces are coplanar within tolerance
            if (pmp::dot(p0.normal, p1.normal) > COPLANAR_NORMAL_TOLERANCE &&
                std::abs(p0.d - p1.d) < COPLANAR_DIST_TOLERANCE) {
                uf.unite(f0.idx(), f1.idx());
            }
        }
    }

    // 3. Aggregate concavity properties for the clustered sets
    // If any face in the coplanar patch is concave, the whole plane is treated as concave.
    std::vector<bool> isSetConcave(numFaces, false);
    for (auto face : state.mesh.faces()) {
        if (!validPlane[face.idx()]) continue;
        int root = uf.find(face.idx());
        if (isConcaveFace[face.idx()]) {
            isSetConcave[root] = true;
        }
    }

    // 4. Extract unique supporting planes (only 1 per disjoint set)
    std::vector<Plane> concavePlanes;
    std::vector<Plane> convexPlanes;
    state.supportPlanes.clear();

    for (auto face : state.mesh.faces()) {
        if (!validPlane[face.idx()]) continue;

        // Only add the plane if this face is the "root" of its coplanar cluster
        if (uf.find(face.idx()) == static_cast<int>(face.idx())) {
            if (isSetConcave[face.idx()]) {
                concavePlanes.push_back(facePlanes[face.idx()]);
            } else {
                convexPlanes.push_back(facePlanes[face.idx()]);
            }
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


/**
 * Performs one step of the kernel generation process. The intermediate kernel is cut at the current support plane,
 * stored in AppState, and the visuals are updated in Polyscope, if `updateVisuals` is set. Checks for termination
 * conditions and advances the current plane index.
 */
void step_kernel(AppState& state, bool updateVisuals = true) {
    if (!state.isSteppingKernel || state.supportPlanes.empty()) return;

    print::info("Kernel Generation: Processing plane " + std::to_string(state.currentPlaneIdx) + " / " + std::to_string(state.supportPlanes.size()));

    if (static_cast<size_t>(state.currentPlaneIdx) >= state.supportPlanes.size()) {
        print::info("Kernel Generation: All planes processed.");
        state.isSteppingKernel = false;

        // Final visual update
        if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
        state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
        state.kSMesh->setSurfaceColor(constants::colors::kernel);
        state.kSMesh->setTransparency(constants::transparencies::kernel);

        return;
    }

    if (state.kHat.is_empty()) {
        print::info("Kernel Generation: Kernel is empty.");
        state.isSteppingKernel = false;

        // Final visual update
        if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
        state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
        state.kSMesh->setSurfaceColor(constants::colors::kernel);
        state.kSMesh->setTransparency(constants::transparencies::kernel);

        return;
    }

    Plane& plane = state.supportPlanes[state.currentPlaneIdx];

    // TODO: Remove this
    plane.d += EPSILON * 5.0f;  // add a small offset to ensure we don't run into numerical issues with coplanar faces
    
    // Visualize the current support plane
    if (updateVisuals) {
        mesh_utils::visualize_cut_plane(state, plane);
    }

    // Perform the cut
    if (state.selectedCutAlgorithm == CutAlgorithm::Standard) {
        mesh_utils::cut_at_plane(state, state.kHat, plane, state.updateVisuals);
    } else {
        mesh_utils::cut_at_plane_linear_search(state, state.kHat, plane, state.updateVisuals);
    }
    
    state.currentPlaneIdx++;

    // Update the kernel surface mesh in Polyscope
    if (updateVisuals) {
        if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
        state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
        state.kSMesh->setSurfaceColor(constants::colors::kernel);
        state.kSMesh->setTransparency(constants::transparencies::kernel);
    }
}

/**
 * Executes the full kernel generation process.
 */
void generate_kernel(AppState& state) {
    init_kernel_stepping(state);
    while (state.isSteppingKernel) {
        step_kernel(state);
    }

    // Final visual update
    if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
    state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
    state.kSMesh->setSurfaceColor(constants::colors::kernel);
    state.kSMesh->setTransparency(constants::transparencies::kernel);
}