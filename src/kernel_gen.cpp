#include <integer-plane-geometry/classify.hh>
#include <integer-plane-geometry/are_parallel.hh>
#include <omp.h>
#include <chrono>

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


/**
 *  Given a bounding box, construct a surface quad mesh representing its axis-aligned bounding box (AABB).
 */
pmp::SurfaceMesh construct_aabb_mesh(pmp::BoundingBox& bbox) {
    pmp::SurfaceMesh aabb;
    auto exactPoints = aabb.add_vertex_property<ExactPoint>("v:exact_pos");
    auto exactPlanes = aabb.add_face_property<ExactPlane>("f:exact_plane");

    pmp::Point min = bbox.min();
    pmp::Point max = bbox.max();

    auto add_exact_vertex = [&](pmp::Point p) {
        auto v = aabb.add_vertex(p);
        tg::ipos3 ipos(
            static_cast<int64_t>(p[0] * globalSettings::scaleFactor),
            static_cast<int64_t>(p[1] * globalSettings::scaleFactor),
            static_cast<int64_t>(p[2] * globalSettings::scaleFactor)
        );
        exactPoints[v] = ExactPoint(ipos);
        return v;
    };

    // Construct the eight vertices of the AABB
    auto v0 = add_exact_vertex(pmp::Point(min[0], min[1], min[2]));
    auto v1 = add_exact_vertex(pmp::Point(max[0], min[1], min[2]));
    auto v2 = add_exact_vertex(pmp::Point(max[0], max[1], min[2]));
    auto v3 = add_exact_vertex(pmp::Point(min[0], max[1], min[2]));
    auto v4 = add_exact_vertex(pmp::Point(min[0], min[1], max[2]));
    auto v5 = add_exact_vertex(pmp::Point(max[0], min[1], max[2]));
    auto v6 = add_exact_vertex(pmp::Point(max[0], max[1], max[2]));
    auto v7 = add_exact_vertex(pmp::Point(min[0], max[1], max[2]));

    // Helper to bind faces and compute their exact supporting planes
    auto add_exact_face = [&](std::vector<pmp::Vertex> vertices) {
        auto f = aabb.add_face(vertices);
        auto vA = exactPoints[vertices[0]];
        auto vB = exactPoints[vertices[1]];
        auto vC = exactPoints[vertices[2]];

        // w=1 for initial AABB points, so x,y,z are exact unscaled integer coordinates
        tg::pos<3, ExactGeom::pos_scalar_t> pA(int64_t(vA.x), int64_t(vA.y), int64_t(vA.z));
        tg::pos<3, ExactGeom::pos_scalar_t> pB(int64_t(vB.x), int64_t(vB.y), int64_t(vB.z));
        tg::pos<3, ExactGeom::pos_scalar_t> pC(int64_t(vC.x), int64_t(vC.y), int64_t(vC.z));

        exactPlanes[f] = ExactPlane::from_points(pA, pB, pC);
    };
    
    // Construct the six faces of the AABB
    add_exact_face({v0, v3, v2, v1}); // Bottom
    add_exact_face({v4, v5, v6, v7}); // Top
    add_exact_face({v0, v1, v5, v4}); // Front
    add_exact_face({v2, v3, v7, v6}); // Back
    add_exact_face({v3, v0, v4, v7}); // Left
    add_exact_face({v1, v2, v6, v5}); // Right

    return aabb;
}

/**
 * Classifies the integer AABB of the intermediate kernel against the given supporting plane.
 * Returns -1 if the AABB is fully in the negative half-space (can skip cut).
 * Returns 1 if the AABB is fully in the positive half-space (kernel is destroyed).
 * Returns 0 if the AABB intersects the plane (cut is needed).
 */
int classify_aabb(const AppState& state, const ExactPlane& exactPlane) {
    // bits_plane_d + 1 handles the max potential sum without overflow (see paper)
    constexpr int bits_out = ExactGeom::bits_plane_d + 1;

    auto c_x = state.aabb_max[0] + state.aabb_min[0];
    auto c_y = state.aabb_max[1] + state.aabb_min[1];
    auto c_z = state.aabb_max[2] + state.aabb_min[2];
    auto s_x = state.aabb_max[0] - state.aabb_min[0];
    auto s_y = state.aabb_max[1] - state.aabb_min[1];
    auto s_z = state.aabb_max[2] - state.aabb_min[2];

    auto d2 = exactPlane.d << 1;  // multiply by 2 to avoid fraction
    auto dot_c = ipg::mul<bits_out>(c_x, exactPlane.a) + ipg::mul<bits_out>(c_y, exactPlane.b) + ipg::mul<bits_out>(c_z, exactPlane.c);
    auto dot_s = ipg::mul<bits_out>(s_x, ipg::abs(exactPlane.a)) + ipg::mul<bits_out>(s_y, ipg::abs(exactPlane.b)) + ipg::mul<bits_out>(s_z, ipg::abs(exactPlane.c));

    auto max_value = dot_c + dot_s + d2;
    auto min_value = dot_c - dot_s + d2;

    if (tg::detail::less_than_zero(max_value) || (max_value == 0)) return -1;
    if (!tg::detail::less_than_zero(min_value) && (min_value != 0)) return 1;
    return 0;
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

    // * Intialize AABB tracking for fast intersection tests
    for (int i = 0; i < 3; ++i) {
        state.aabb_min[i] = std::numeric_limits<int64_t>::max();
        state.aabb_max[i] = std::numeric_limits<int64_t>::lowest();
    }
    auto exactPoints_k = state.kHat.get_vertex_property<ExactPoint>("v:exact_pos");
    for (auto v : state.kHat.vertices()) {
        ExactPoint p = exactPoints_k[v];
        for (int i = 0; i < 3; ++i) {
            int64_t val = static_cast<int64_t>(p.comp(i));  // w=1 for initial AABB
            if (val < state.aabb_min[i]) {
                state.aabb_min[i] = val;
                state.aabb_v_min[i] = v;
            }
            if (val > state.aabb_max[i]) {
                state.aabb_max[i] = val;
                state.aabb_v_max[i] = v;
            }
        }
    }
    state.skippedCuts = 0;  // reset

    // * Identify concave faces
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

    auto exactPoints = state.mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    auto exactPlanes = state.mesh.get_face_property<ExactPlane>("f:exact_plane");

    for (auto face : state.mesh.faces()) {
        auto it = state.mesh.vertices(face).begin();
        ExactPoint vA = exactPoints[*it]; ++it;
        ExactPoint vB = exactPoints[*it]; ++it;
        ExactPoint vC = exactPoints[*it];

        tg::pos<3, ExactGeom::pos_scalar_t> pA(int64_t(vA.x), int64_t(vA.y), int64_t(vA.z));
        tg::pos<3, ExactGeom::pos_scalar_t> pB(int64_t(vB.x), int64_t(vB.y), int64_t(vB.z));
        tg::pos<3, ExactGeom::pos_scalar_t> pC(int64_t(vC.x), int64_t(vC.y), int64_t(vC.z));

        exactPlanes[face] = ExactPlane::from_points(pA, pB, pC);

        if (exactPlanes[face].is_valid()) {
            auto dp = exactPlanes[face].to_dplane();
            facePlanes[face.idx()] = Plane{
                pmp::vec3(dp.normal.x, dp.normal.y, dp.normal.z),
                static_cast<float>(-dp.dis / globalSettings::scaleFactor)
            };
            validPlane[face.idx()] = true;
        }
    }

    // 2. Group connected coplanar faces using Union-Find
    UnionFind uf(numFaces);

    for (auto edge : state.mesh.edges()) {
        if (state.mesh.is_boundary(edge)) continue;

        // Acquire the two faces adjacent to this edge via the two halfedges
        pmp::Face f0 = state.mesh.face(state.mesh.halfedge(edge, 0));
        pmp::Face f1 = state.mesh.face(state.mesh.halfedge(edge, 1));

        if (validPlane[f0.idx()] && validPlane[f1.idx()]) {
            const ExactPlane& ep0 = exactPlanes[f0];
            const ExactPlane& ep1 = exactPlanes[f1];

            bool are_identical = false;
            if (ipg::are_parallel(ep0, ep1)) {
                // Checks if adjacent faces have identical supporting planes
                if (!tg::is_zero(ep0.a)) {
                    are_identical = (ipg::mul<192>(ep0.a, ep1.d) == ipg::mul<192>(ep1.a, ep0.d));
                } else if (!tg::is_zero(ep0.b)) {
                    are_identical = (ipg::mul<192>(ep0.b, ep1.d) == ipg::mul<192>(ep1.b, ep0.d));
                } else {
                    are_identical = (ipg::mul<192>(ep0.c, ep1.d) == ipg::mul<192>(ep1.c, ep0.d));
                }
            }

            if (are_identical) {
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
    std::vector<ExactPlane> exactConcavePlanes;
    std::vector<ExactPlane> exactConvexPlanes;
    state.supportPlanes.clear();
    state.exactSupportPlanes.clear();

    for (auto face : state.mesh.faces()) {
        if (!validPlane[face.idx()]) continue;

        // Only add the plane if this face is the "root" of its coplanar cluster
        if (uf.find(face.idx()) == static_cast<int>(face.idx())) {
            if (isSetConcave[face.idx()]) {
                concavePlanes.push_back(facePlanes[face.idx()]);
                exactConcavePlanes.push_back(exactPlanes[face]);
            } else {
                convexPlanes.push_back(facePlanes[face.idx()]);
                exactConvexPlanes.push_back(exactPlanes[face]);
            }
        }
    }

    // Prioritize concave planes first, then convex planes
    state.supportPlanes.insert(state.supportPlanes.end(), concavePlanes.begin(), concavePlanes.end());
    state.supportPlanes.insert(state.supportPlanes.end(), convexPlanes.begin(), convexPlanes.end());
    state.exactSupportPlanes.insert(state.exactSupportPlanes.end(), exactConcavePlanes.begin(), exactConcavePlanes.end());
    state.exactSupportPlanes.insert(state.exactSupportPlanes.end(), exactConvexPlanes.begin(), exactConvexPlanes.end());

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
    ExactPlane& exactPlane = state.exactSupportPlanes[state.currentPlaneIdx];

    // Fast AABB Intersection check
    int aabb_class = classify_aabb(state, exactPlane);
    if (aabb_class == -1) {
        // fully in negative half-space, skip cut
        print::info("AABB Check: Plane does not intersect. Skipping cut.");
        state.skippedCuts++;
        state.currentPlaneIdx++;
        return;
    } else if (aabb_class == 1) {
        // fully in positive half-space, kernel is destroyed
        print::info("AABB Check: Kernel entirely discarded. Kernel is empty.");
        state.kHat.clear();
        state.currentPlaneIdx = state.supportPlanes.size();  // terminate
        return;
    }
    
    // Visualize the current support plane
    if (updateVisuals) {
        mesh_utils::visualize_cut_plane(state, plane);
    }

    // Perform the cut
    mesh_utils::cut_at_plane_exact(state, state.kHat, plane, state.exactSupportPlanes[state.currentPlaneIdx], state.updateVisuals);
    
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


void generate_kernel_parallel(AppState& state) {
    if (!state.meshLoaded || state.mesh.is_empty()) return;

    print::info("Starting parallel kernel generation...");
    auto start_time = std::chrono::high_resolution_clock::now();

    // Genus early termination
    int euler = state.mesh.n_vertices() - state.mesh.n_edges() + state.mesh.n_faces();
    if (euler < 2) {
        polyscope::info("Mesh has genus > 0. Kernel is empty.");
        state.kHat.clear();
        if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
        return;
    }

    // Convex mesh early termination
    std::vector<bool> isConcaveFace = mesh_utils::identify_concave_faces(state.mesh);

    if (std::none_of(isConcaveFace.begin(), isConcaveFace.end(), [](bool v) { return v; })) {
        polyscope::info("Mesh has no concave faces. Kernel is the mesh itself.");
        state.kHat = state.mesh;
        if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
        state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
        state.kSMesh->setSurfaceColor(constants::colors::kernel);
        state.kSMesh->setTransparency(constants::transparencies::kernel);
        return;
    }

    pmp::BoundingBox bbox = pmp::bounds(state.mesh);
    pmp::Point center = bbox.center();

    state.skippedCuts = 0;
    std::atomic<int> local_skipped_cuts{0};  // Thread-safe counter

    // Identify valid planes and group coplanar faces
    size_t numFaces = state.mesh.n_faces();
    std::vector<Plane> facePlanes(numFaces);
    std::vector<bool> validPlane(numFaces, false);
    auto exactPoints = state.mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    auto exactPlanes = state.mesh.get_face_property<ExactPlane>("f:exact_plane");

    for (auto face : state.mesh.faces()) {
        auto it = state.mesh.vertices(face).begin();
        ExactPoint vA = exactPoints[*it]; ++it;
        ExactPoint vB = exactPoints[*it]; ++it;
        ExactPoint vC = exactPoints[*it];
        
        tg::pos<3, ExactGeom::pos_scalar_t> pA(int64_t(vA.x), int64_t(vA.y), int64_t(vA.z));
        tg::pos<3, ExactGeom::pos_scalar_t> pB(int64_t(vB.x), int64_t(vB.y), int64_t(vB.z));
        tg::pos<3, ExactGeom::pos_scalar_t> pC(int64_t(vC.x), int64_t(vC.y), int64_t(vC.z));
        exactPlanes[face] = ExactPlane::from_points(pA, pB, pC);

        if (exactPlanes[face].is_valid()) {
            auto dp = exactPlanes[face].to_dplane();
            facePlanes[face.idx()] = Plane{
                pmp::vec3(dp.normal.x, dp.normal.y, dp.normal.z),
                static_cast<float>(-dp.dis / globalSettings::scaleFactor)
            };
            validPlane[face.idx()] = true;
        }
    }

    UnionFind uf(numFaces);
    for (auto edge : state.mesh.edges()) {
        if (state.mesh.is_boundary(edge)) continue;
        pmp::Face f0 = state.mesh.face(state.mesh.halfedge(edge, 0));
        pmp::Face f1 = state.mesh.face(state.mesh.halfedge(edge, 1));
        if (validPlane[f0.idx()] && validPlane[f1.idx()]) {
            const ExactPlane& ep0 = exactPlanes[f0];
            const ExactPlane& ep1 = exactPlanes[f1];
            bool are_identical = false;
            if (ipg::are_parallel(ep0, ep1)) {
                if (!tg::is_zero(ep0.a)) {
                    are_identical = (ipg::mul<192>(ep0.a, ep1.d) == ipg::mul<192>(ep1.a, ep0.d));
                } else if (!tg::is_zero(ep0.b)) {
                    are_identical = (ipg::mul<192>(ep0.b, ep1.d) == ipg::mul<192>(ep1.b, ep0.d));
                } else {
                    are_identical = (ipg::mul<192>(ep0.c, ep1.d) == ipg::mul<192>(ep1.c, ep0.d));
                }
            }
            if (are_identical) uf.unite(f0.idx(), f1.idx());
        }
    }

    // Identify concave faces and aggregate concavity for the clustered sets
    std::vector<bool> isSetConcave(numFaces, false);
    for (auto face : state.mesh.faces()) {
        if (!validPlane[face.idx()]) continue;
        int root = uf.find(face.idx());
        if (isConcaveFace[face.idx()]) {
            isSetConcave[root] = true;
        }
    }

    // * Divide planes into octants based on root face centroids
    std::vector<Plane> octant_concave_planes[8];
    std::vector<ExactPlane> octant_concave_exact[8];
    std::vector<Plane> octant_convex_planes[8];
    std::vector<ExactPlane> octant_convex_exact[8];

    for (auto face : state.mesh.faces()) {
        if (!validPlane[face.idx()]) continue;

        // Only process the root face of each coplanar cluster
        if (uf.find(face.idx()) == static_cast<int>(face.idx())) {
            // Calculate the centroid of the face to determine its octant
            pmp::Point centroid(0,0,0);
            int v_count = 0;
            for (auto v : state.mesh.vertices(face)) {
                centroid += state.mesh.position(v);
                v_count++;
            }
            centroid /= static_cast<float>(v_count);

            int octant = 0;
            if (centroid[0] > center[0]) octant |= 1;
            if (centroid[1] > center[1]) octant |= 2;
            if (centroid[2] > center[2]) octant |= 4;

            // Route to appropriate octant list based on concavity
            if (isSetConcave[face.idx()]) {
                octant_concave_planes[octant].push_back(facePlanes[face.idx()]);
                octant_concave_exact[octant].push_back(exactPlanes[face]);
            } else {
                octant_convex_planes[octant].push_back(facePlanes[face.idx()]);
                octant_convex_exact[octant].push_back(exactPlanes[face]);
            }
        }
    }

    // Merge concave and convex planes for each octant, prioritizing concave planes
    std::vector<Plane> octant_planes[8];
    std::vector<ExactPlane> octant_exact[8];

    for (int i = 0; i < 8; ++i) {
        octant_planes[i].insert(octant_planes[i].end(), octant_concave_planes[i].begin(), octant_concave_planes[i].end());
        octant_planes[i].insert(octant_planes[i].end(), octant_convex_planes[i].begin(), octant_convex_planes[i].end());
        octant_exact[i].insert(octant_exact[i].end(), octant_concave_exact[i].begin(), octant_concave_exact[i].end());
        octant_exact[i].insert(octant_exact[i].end(), octant_convex_exact[i].begin(), octant_convex_exact[i].end());
    }

    // * Process each octant in parallel
    std::vector<pmp::SurfaceMesh> local_kernels(8);
    bool empty_kernel_detected = false;

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < 8; ++i) {
        if (empty_kernel_detected) continue;

        AppState local_state;
        local_state.kHat = construct_aabb_mesh(bbox);

        // Initialize local AABB tracking
        for (int j = 0; j < 3; ++j) {
            local_state.aabb_min[j] = std::numeric_limits<int64_t>::max();
            local_state.aabb_max[j] = std::numeric_limits<int64_t>::lowest();
        }
        auto exactPoints_k = local_state.kHat.get_vertex_property<ExactPoint>("v:exact_pos");
        for (auto v : local_state.kHat.vertices()) {
            ExactPoint p = exactPoints_k[v];
            for (int j = 0; j < 3; ++j) {
                int64_t val = static_cast<int64_t>(p.comp(j));
                if (val < local_state.aabb_min[j]) { local_state.aabb_min[j] = val; local_state.aabb_v_min[j] = v; }
                if (val > local_state.aabb_max[j]) { local_state.aabb_max[j] = val; local_state.aabb_v_max[j] = v; }
            }
        }

        for (size_t p = 0; p < octant_planes[i].size(); ++p) {
            int aabb_class = classify_aabb(local_state, octant_exact[i][p]);
            if (aabb_class == -1) {
                local_skipped_cuts++;
                continue;
            };
            if (aabb_class == 1) {
                local_state.kHat.clear();
                empty_kernel_detected = true;
                break;
            }

            mesh_utils::cut_at_plane_exact(local_state, local_state.kHat, octant_planes[i][p], octant_exact[i][p], false);
            if (local_state.kHat.is_empty()) {
                empty_kernel_detected = true;
                break;
            }
        }
        local_kernels[i] = std::move(local_state.kHat);
    }

    state.skippedCuts = local_skipped_cuts.load();

    // * Merge surviving planes and compute final intersection
    if (empty_kernel_detected) {
        state.kHat.clear();
    } else {
        std::vector<Plane> final_planes;
        std::vector<ExactPlane> final_exact;

        for (int i = 0; i < 8; ++i) {
            auto exact_props = local_kernels[i].get_face_property<ExactPlane>("f:exact_plane");
            for (auto f : local_kernels[i].faces()) {
                ExactPlane ep = exact_props[f];
                auto dp = ep.to_dplane();
                Plane p{
                    pmp::vec3(dp.normal.x, dp.normal.y, dp.normal.z),
                    static_cast<float>(-dp.dis / globalSettings::scaleFactor)
                };
                final_planes.push_back(p);
                final_exact.push_back(ep);
            }
        }

        // Final sequential cut on original AABB
        state.kHat = construct_aabb_mesh(bbox);
        for (int j = 0; j < 3; ++j) {
            state.aabb_min[j] = std::numeric_limits<int64_t>::max();
            state.aabb_max[j] = std::numeric_limits<int64_t>::lowest();
        }
        auto exactPoints_k = state.kHat.get_vertex_property<ExactPoint>("v:exact_pos");
        for (auto v : state.kHat.vertices()) {
            ExactPoint pt = exactPoints_k[v];
            for (int j = 0; j < 3; ++j) {
                int64_t val = static_cast<int64_t>(pt.comp(j));
                if (val < state.aabb_min[j]) { state.aabb_min[j] = val; state.aabb_v_min[j] = v; }
                if (val > state.aabb_max[j]) { state.aabb_max[j] = val; state.aabb_v_max[j] = v; }
            }
        }

        for (size_t p = 0; p < final_planes.size(); ++p) {
            int aabb_class = classify_aabb(state, final_exact[p]);
            if (aabb_class == -1) continue;
            if (aabb_class == 1) {
                state.kHat.clear();
                break;
            }
            mesh_utils::cut_at_plane_exact(state, state.kHat, final_planes[p], final_exact[p], false);
            if (state.kHat.is_empty()) break;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    state.lastComputeTime = elapsed.count();
    
    // * Update visuals
    state.isSteppingKernel = false;
    if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
    state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
    state.kSMesh->setSurfaceColor(constants::colors::kernel);
    state.kSMesh->setTransparency(constants::transparencies::kernel);
}