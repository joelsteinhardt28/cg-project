#include "mesh_utils.hpp"

#include <iostream>
#include <random>
#include <algorithm>
#include <integer-plane-geometry/classify.hh>
#include <integer-plane-geometry/intersect.hh>

#include "polyscope/curve_network.h"

#include <pmp/bounding_box.h>
#include <pmp/algorithms/utilities.h>
#include <pmp/exceptions.h>

namespace mesh_utils {

// * HELPER FUNCTIONS * //

/*
 * Registers a PMP surface mesh in Polyscope using the vertex positions and face indices of the given PMP mesh. The surface
 * mesh is registered with the name `name`.
 */
polyscope::SurfaceMesh* register_pmp_mesh(const std::string& name, const pmp::SurfaceMesh& mesh) {
    print::debug("Registering PMP mesh with Polyscope: " + name);
    std::vector<Point> vertices;
    vertices.reserve(mesh.n_vertices());
    for (auto v : mesh.vertices()) {
        auto p = mesh.position(v);
        vertices.push_back(p);
    }

    std::vector<Face> faces;
    faces.reserve(mesh.n_faces());
    for (auto f : mesh.faces()) {
        std::vector<size_t> face_indices;
        for (auto v : mesh.vertices(f)) {
            face_indices.push_back(v.idx());
        }
        faces.push_back(face_indices);
    }

    return polyscope::registerSurfaceMesh(name, vertices, faces);
}

/*
 * Registers a PMP point cloud in Polyscope using the vertex positions of the given PMP mesh. The point cloud is registered
 * with the name `name`.
 */
polyscope::PointCloud* register_pmp_pc(const std::string& name, const pmp::SurfaceMesh& mesh) {
    print::info("Registering PMP point cloud with Polyscope: " + name);
    std::vector<Point> vertices;
    vertices.reserve(mesh.n_vertices());
    for (auto v : mesh.vertices()) {
        auto p = mesh.position(v);
        vertices.push_back(p);
    }
    return polyscope::registerPointCloud(name, vertices);
}

/*
 * Registers or updates the bounding box and its visualization for `state.mesh`.
 */
void register_bbox(AppState& state) {
    if (!state.meshLoaded) return;

    pmp::BoundingBox bbox = pmp::bounds(state.mesh);

    state.bboxVertices = {
        pmp::vec3({bbox.min()[0], bbox.min()[1], bbox.min()[2]}), // 0
        pmp::vec3({bbox.max()[0], bbox.min()[1], bbox.min()[2]}), // 1
        pmp::vec3({bbox.max()[0], bbox.max()[1], bbox.min()[2]}), // 2
        pmp::vec3({bbox.min()[0], bbox.max()[1], bbox.min()[2]}), // 3
        pmp::vec3({bbox.min()[0], bbox.min()[1], bbox.max()[2]}), // 4
        pmp::vec3({bbox.max()[0], bbox.min()[1], bbox.max()[2]}), // 5
        pmp::vec3({bbox.max()[0], bbox.max()[1], bbox.max()[2]}), // 6
        pmp::vec3({bbox.min()[0], bbox.max()[1], bbox.max()[2]})  // 7
    };

    std::vector<std::array<size_t, 2>> bboxEdges = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom face
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top face
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // vertical edges
    };

    // Visualize bounding box
    auto* bboxCN = polyscope::registerCurveNetwork(std::string(constants::polyNames::bbox), state.bboxVertices, bboxEdges);
    bboxCN->setRadius(constants::otherVisuals::bboxRadius);
    bboxCN->setColor(constants::colors::bbox);
}


/*
 * Visualizes the given plane as a quad that fills the bbox of the mesh. There can only be one cutting plane
 * visualized at a time. It is registered as `constants::polyNames::cutPlane` in Polyscope. The normal of 
 * the plane is also visualized as a curve network registered as `constants::polyNames::cutPlaneNormal`.
 */
void visualize_cut_plane(AppState& state, const Plane& plane) {
    if (state.bboxVertices.empty()) return;

    // Calculate bbox limits
    Point minP = state.bboxVertices[0];
    Point maxP = state.bboxVertices[0];
    for (const auto& v : state.bboxVertices) {
        for (int i = 0; i < 3; i++) {
            minP[i] = std::min(minP[i], v[i]);
            maxP[i] = std::max(maxP[i], v[i]);
        }
    }

    glm::vec3 normal(plane.normal[0], plane.normal[1], plane.normal[2]);
    
    // Find a center point for the quad. We project the center of the BBox onto the plane.
    Point bboxCenter = (minP + maxP) * 0.5f;
    float dist = plane.distance(bboxCenter);
    Point projectedCenter = bboxCenter - dist * plane.normal;
    glm::vec3 center(projectedCenter[0], projectedCenter[1], projectedCenter[2]);

    glm::vec3 helper = (std::abs(normal.y) < 0.99) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
    glm::vec3 tangent = glm::normalize(glm::cross(normal, helper));
    glm::vec3 bitangent = glm::normalize(glm::cross(normal, tangent));
    
    float scale = glm::distance(glm::vec3(minP[0], minP[1], minP[2]), 
                                     glm::vec3(maxP[0], maxP[1], maxP[2])) * 2.0f;

    std::vector<glm::vec3> polygon = {
        center + scale * (tangent + bitangent),
        center + scale * (-tangent + bitangent),
        center + scale * (-tangent - bitangent),
        center + scale * (tangent - bitangent)
    };
    
    // Clip the polygon against the 6 planes of the Bounding Box
    auto clip = [&](std::vector<glm::vec3>& poly, int axis, float val, bool isMax) {
        std::vector<glm::vec3> nextPoly;
        if (poly.empty()) return nextPoly;

        for (size_t i = 0; i < poly.size(); i++) {
            glm::vec3 p1 = poly[i];
            glm::vec3 p2 = poly[(i + 1) % poly.size()];

            float v1 = p1[axis];
            float v2 = p2[axis];

            auto inside = [&](float v) { return isMax ? (v <= val) : (v >= val); };

            if (inside(v1)) {
                if (inside(v2)) {
                    nextPoly.push_back(p2);
                } else {
                    float t = (val - v1) / (v2 - v1);
                    nextPoly.push_back(p1 + t * (p2 - p1));
                }
            } else if (inside(v2)) {
                float t = (val - v1) / (v2 - v1);
                nextPoly.push_back(p1 + t * (p2 - p1));
                nextPoly.push_back(p2);
            }
        }
        return nextPoly;
    };

    // Sequentially clip against all 6 sides
    for (int i = 0; i < 3; i++) {
        polygon = clip(polygon, i, minP[i], false); // Clip min side
        polygon = clip(polygon, i, maxP[i], true);  // Clip max side
    }

    if (polygon.empty()) return;

    std::vector<std::vector<size_t>> faces;
    std::vector<size_t> face;
    for(size_t i = 0; i < polygon.size(); ++i) face.push_back(i);
    faces.push_back(face);

    auto* planeMesh = polyscope::registerSurfaceMesh(std::string(constants::polyNames::cutPlane), polygon, faces);
    planeMesh->setSurfaceColor(constants::colors::cutPlane);
    planeMesh->setTransparency(constants::transparencies::cutPlane);

    // Visualize plane normal
    std::vector<glm::vec3> normalVertices = {
        center,
        center + normal * (scale * 0.1f)
    };
    std::vector<std::array<size_t, 2>> normalEdges = {{0, 1}};
    auto* normalCN = polyscope::registerCurveNetwork(std::string(constants::polyNames::cutPlaneNormal), normalVertices, normalEdges);
    normalCN->setColor(constants::colors::cutPlaneNormal);
    normalCN->setRadius(constants::otherVisuals::normalRadius);
}


/*
 * Generates a random plane that intersects the bounding box of `state.mesh` and visualizes it. The plane is stored in
 * `state.activeCutPlane` and can be used for cutting the mesh.
 */
void generate_random_bbox_plane(AppState& state) {
    if (state.bboxVertices.empty()) return;

    // Calculate bbox limits
    Point minP = state.bboxVertices[0];
    Point maxP = state.bboxVertices[0];
    for (const auto& v : state.bboxVertices) {
        for (int i = 0; i < 3; i++) {
            minP[i] = std::min(minP[i], v[i]);
            maxP[i] = std::max(maxP[i], v[i]);
        }
    }

    // Setup random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(minP[0], maxP[0]);
    std::uniform_real_distribution<float> distY(minP[1], maxP[1]);
    std::uniform_real_distribution<float> distZ(minP[2], maxP[2]);
    std::uniform_real_distribution<float> distUnit(-1.0f, 1.0f);

    // Sample center
    glm::vec3 center(distX(gen), distY(gen), distZ(gen));
    glm::vec3 normal{distUnit(gen), distUnit(gen), distUnit(gen)};
    if (glm::length(normal) < 1e-5) normal = {0.0f, 0.0f, 1.0f}; // fallback normal
    normal = glm::normalize(normal);

    // Store active cut plane
    state.activeCutPlane.normal = pmp::Point(normal.x, normal.y, normal.z);
    state.activeCutPlane.d = -pmp::dot(state.activeCutPlane.normal, pmp::Point(center.x, center.y, center.z));
    state.hasActiveCutPlane = true;

    visualize_cut_plane(state, state.activeCutPlane);
}


/**
 * Given a mesh, returns a boolean vector indicating which faces are concave.
 * The function evaluates the signed volume formed by the current face and its opposite face across each edge.
 * If the volume is positive, it indicates that the opposite face is above the plane of the current face, 
 * suggesting a concave configuation.
 */
std::vector<bool> identify_concave_faces(const pmp::SurfaceMesh& mesh) {
    std::vector<bool> isConcave(mesh.n_faces(), false);

    auto exact_points = mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    auto exact_planes = mesh.get_face_property<ExactPlane>("f:exact_plane");

    for (auto e : mesh.edges()) {
        if (mesh.is_boundary(e)) continue;

        pmp::Halfedge he0 = mesh.halfedge(e, 0);
        pmp::Halfedge he1 = mesh.halfedge(e, 1);
        pmp::Face f0 = mesh.face(he0);
        pmp::Face f1 = mesh.face(he1);

        // Find a vertex in f1 that does not share the edge e to test against the plane of f0
        pmp::Vertex opposite;
        for (auto v : mesh.vertices(f1)) {
            if (v != mesh.vertex(e, 0) && v != mesh.vertex(e, 1)) {
                opposite = v;
                break;
            }
        }

        if (exact_points && exact_planes) {
            // For a test vertex and the plane of f0
            ExactPoint p_test = exact_points[opposite];
            ExactPlane plane_f0 = exact_planes[f0];

            // classify returns +1, 0, or -1 exactly.
            if (ipg::classify(p_test, plane_f0) > 0) {
                isConcave[f0.idx()] = true;
                isConcave[f1.idx()] = true;
            }
        } else {
            // Fallback to floating-point classifier if exact properties are not available
            auto it = mesh.vertices(f0).begin();
            pmp::Point p0 = mesh.position(*it); ++it;
            pmp::Point p1 = mesh.position(*it); ++it;
            pmp::Point p2 = mesh.position(*it);
            pmp::vec3 n_f0 = pmp::cross(p1 - p0, p2 - p0);

            pmp::Point p_test = mesh.position(opposite);
            float det = pmp::dot(n_f0, p_test - p0);  // Scalar triple product

            if (det > EPSILON) {
                isConcave[f0.idx()] = true;
                isConcave[f1.idx()] = true;
            }
        }
    }

    return isConcave;
}


// * IMPLEMENTATION OF MESH-PLANE CUTTING * //

/**
 * Find an edge that crosses the given plane.
 */
pmp::Halfedge edge_descent(pmp::SurfaceMesh& mesh, const Plane& plane) {
    if (mesh.is_empty()) return pmp::Halfedge();

    // Lambda helper to check if a vertex is inside or on the plane
    auto is_inside_or_on = [&](pmp::Vertex v) {
        return plane.distance(mesh.position(v)) <= EPSILON;
    };

    pmp::Vertex current_v = *mesh.vertices_begin();  // start from an arbitrary vertex
    float current_dist = plane.distance(mesh.position(current_v));
    bool current_state = is_inside_or_on(current_v);

    // Keep track of visited vertices to prevent infinite loops
    std::vector<bool> visited(mesh.n_vertices(), false);

    while(current_v.is_valid()) {
        visited[current_v.idx()] = true;

        pmp::Vertex bestNeighbor;
        float min_abs_dist = std::abs(current_dist);
        bool foundCloser = false;

        for (auto he : mesh.halfedges(current_v)) {
            pmp::Vertex neighbor = mesh.to_vertex(he);

            if (current_state != is_inside_or_on(neighbor)) {
                return he; // Found edge crossing the plane
            }

            // Look for direction which brings us closer to the plane
            if (!visited[neighbor.idx()]) {
                float neighbor_dist = plane.distance(mesh.position(neighbor));
                if (std::abs(neighbor_dist) < min_abs_dist) {
                    min_abs_dist = std::abs(neighbor_dist);
                    bestNeighbor = neighbor;
                    foundCloser = true;
                }
            }
        }

        // No vertex brings us closer to the plane, so we are at a local minimum
        if (!foundCloser) {
            print::warning("Edge descent hit a local minimum.");
            return pmp::Halfedge();
        }

        current_v = bestNeighbor;
        current_dist = plane.distance(mesh.position(current_v));
        current_state = is_inside_or_on(current_v);
    }

    return pmp::Halfedge(); // No crossing edge found
}

/**
 * Find an edge that crosses the given plane using ipg exact arithmetics.
 */
pmp::Halfedge edge_descent_exact(pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane) {
    if (mesh.is_empty()) return pmp::Halfedge();
    
    auto exact_points = mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    auto get_class = [&](pmp::Vertex v) {
        if (exact_points) return static_cast<int>(ipg::classify(exact_points[v], exactPlane));
        float d = plane.distance(mesh.position(v));
        return (d > EPSILON) ? 1 : ((d < -EPSILON) ? -1 : 0);
    };

    pmp::Vertex current_v = *mesh.vertices_begin();
    float current_dist = plane.distance(mesh.position(current_v));
    int current_class = get_class(current_v);
    
    std::vector<bool> visited(mesh.n_vertices(), false);

    while (current_v.is_valid()) {
        visited[current_v.idx()] = true;
        pmp::Vertex bestNeighbor;
        float best_dist = current_dist;
        bool foundCloser = false;

        for (auto he : mesh.halfedges(current_v)) {
            pmp::Vertex neighbor = mesh.to_vertex(he);
            int neighbor_class = get_class(neighbor);

            // Crossing detected! (Transitions between strictly positive/negative, or touching)
            if ((current_class <= 0 && neighbor_class > 0) || (current_class > 0 && neighbor_class <= 0)) {
                return he; 
            }

            if (!visited[neighbor.idx()]) {
                float neighbor_dist = plane.distance(mesh.position(neighbor));
                
                // Move monotonically towards the plane (dist == 0)
                bool moves_closer = false;
                if (current_dist > 0 && neighbor_dist < best_dist) moves_closer = true;
                if (current_dist < 0 && neighbor_dist > best_dist) moves_closer = true;

                if (moves_closer) {
                    best_dist = neighbor_dist;
                    bestNeighbor = neighbor;
                    foundCloser = true;
                }
            }
        }

        if (!foundCloser) break; // Local extremum reached without crossing

        current_v = bestNeighbor;
        current_dist = best_dist;
        current_class = get_class(current_v);
    }
    
    return pmp::Halfedge(); // No crossing found (plane completely misses)
}


/**
 * Performs a cut of the mesh at the given plane. The positive half-space of the plane is discarded.
 * The mesh is required to be convex and needs to have the exact vertex positions and face planes stored as properties
 * `v:exact_pos` and `f:exact_plane`. The Polyscope visualization will be updated after the cut.
 */
void cut_at_plane_exact(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane, bool updateVisuals) {
    if (mesh.is_empty()) return;
    if (updateVisuals) visualize_cut_plane(state, plane);

    auto exact_points = mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    auto exact_planes = mesh.get_face_property<ExactPlane>("f:exact_plane");

    // Perform edge descent to find an edge that crosses the cutting plane
    pmp::Halfedge start_he = edge_descent_exact(mesh, plane, exactPlane);

    if (!start_he.is_valid()) {
        // No crossing edge found, check if mesh is entirely on one side of the plane and exit early if so
        pmp::Vertex first = *mesh.vertices_begin();
        int first_class = ipg::classify(exact_points[first], exactPlane);
        if (first_class > 0) {
            print::info("All vertices are on the positive side of the plane. Discarding kernel.");
            mesh.clear();
        }
        return;
    }

    // Exact Vertex Classification (-1: Keep, 0: On Plane, 1: Discard)
    std::map<pmp::Vertex, int> v_class;
    for (auto v : mesh.vertices()) v_class[v] = ipg::classify(exact_points[v], exactPlane);

    // Check if any AABB extreme vertices are about to be discarded
    bool min_discarded[3] = {false, false, false};
    bool max_discarded[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        if (state.aabb_v_min[i].is_valid() && v_class[state.aabb_v_min[i]] > 0) min_discarded[i] = true;
        if (state.aabb_v_max[i].is_valid() && v_class[state.aabb_v_max[i]] > 0) max_discarded[i] = true;
    }

    // Map kept vertices (and track ones exactly on the plane)
    pmp::SurfaceMesh newMesh;
    auto new_exact_points = newMesh.add_vertex_property<ExactPoint>("v:exact_pos");
    auto new_exact_planes = newMesh.add_face_property<ExactPlane>("f:exact_plane");
    std::map<pmp::Vertex, pmp::Vertex> vertexMap;
    std::map<pmp::Edge, pmp::Vertex> edgeIntersections;
    std::vector<pmp::Vertex> capVertices;

    for (auto v : mesh.vertices()) {
        if (v_class[v] <= 0) {
            auto nv = newMesh.add_vertex(mesh.position(v));
            vertexMap[v] = nv;
            new_exact_points[nv] = exact_points[v];
            if (v_class[v] == 0) capVertices.push_back(nv); 
        }
    }

    // Compute intersections for crossing edges
    for (auto e : mesh.edges()) {
        pmp::Vertex v0 = mesh.vertex(e, 0);
        pmp::Vertex v1 = mesh.vertex(e, 1);
        
        // Only split if strictly crossing (-1 to 1)
        if ((v_class[v0] < 0 && v_class[v1] > 0) || (v_class[v0] > 0 && v_class[v1] < 0)) {
            pmp::Face f0 = mesh.face(mesh.halfedge(e, 0));
            pmp::Face f1 = mesh.face(mesh.halfedge(e, 1));
            
            ExactPlane p0 = exact_planes[f0];
            ExactPlane p1 = exact_planes[f1];
            
            ExactPoint pt = ipg::intersect(p0, p1, exactPlane);
            
            // Convert the exact intersection point to floating-point for mesh visualization
            tg::dpos3 tg_pos = ipg::to_dpos3_fast(pt);
            pmp::Point float_pos(
                static_cast<float>(tg_pos.x / globalSettings::scaleFactor),
                static_cast<float>(tg_pos.y / globalSettings::scaleFactor),
                static_cast<float>(tg_pos.z / globalSettings::scaleFactor)
            );
            
            auto nv = newMesh.add_vertex(float_pos);
            new_exact_points[nv] = pt;
            edgeIntersections[e] = nv;
            capVertices.push_back(nv);
        }
    }

    // Rebuild the clipped faces
    for (auto f : mesh.faces()) {
        std::vector<pmp::Vertex> faceVerts;
        for (auto he : mesh.halfedges(f)) {
            pmp::Vertex v_from = mesh.from_vertex(he);
            pmp::Vertex v_to = mesh.to_vertex(he);
            
            if (v_class[v_from] <= 0) faceVerts.push_back(vertexMap[v_from]);
            
            if ((v_class[v_from] < 0 && v_class[v_to] > 0) || (v_class[v_from] > 0 && v_class[v_to] < 0)) {
                faceVerts.push_back(edgeIntersections[mesh.edge(he)]);
            }
        }

        if (faceVerts.size() >= 3) {
            // Deduplicate safely to protect PMP topology
            faceVerts.erase(std::unique(faceVerts.begin(), faceVerts.end()), faceVerts.end());
            if (faceVerts.size() >= 3 && faceVerts.front() == faceVerts.back()) faceVerts.pop_back();
            
            if (faceVerts.size() >= 3) {
                try {
                    auto nf = newMesh.add_face(faceVerts);
                    new_exact_planes[nf] = exact_planes[f]; // Preserve the exact supporting plane
                } catch (...) {} // Ignore silently degenerate faces at corners
            }
        }
    }

    // Fill the cap face
    if (capVertices.size() >= 3) {
        std::sort(capVertices.begin(), capVertices.end());
        capVertices.erase(std::unique(capVertices.begin(), capVertices.end()), capVertices.end());

        if (capVertices.size() >= 3) {
            // For a convex cut, radial sorting around the centroid guarantees a non-self-intersecting polygon
            pmp::Point centroid(0,0,0);
            for (auto v : capVertices) centroid += newMesh.position(v);
            centroid /= capVertices.size();

            pmp::vec3 n = plane.normal;
            pmp::vec3 u, v_vec;
            pmp::vec3 perp = (std::abs(n[0]) > 0.9f) ? pmp::vec3(0,1,0) : pmp::vec3(1,0,0);
            u = pmp::normalize(pmp::cross(n, perp));
            v_vec = pmp::normalize(pmp::cross(n, u));

            std::sort(capVertices.begin(), capVertices.end(), [&](pmp::Vertex a, pmp::Vertex b) {
                pmp::Point pa = newMesh.position(a) - centroid;
                pmp::Point pb = newMesh.position(b) - centroid;
                return std::atan2(pmp::dot(pa, v_vec), pmp::dot(pa, u)) < std::atan2(pmp::dot(pb, v_vec), pmp::dot(pb, u));
            });

            try {
                auto cap_f = newMesh.add_face(capVertices);
                new_exact_planes[cap_f] = exactPlane; // The cut plane is the new exact plane
            } catch (...) {
                std::reverse(capVertices.begin(), capVertices.end()); // Flip normal and retry
                try {
                    auto cap_f = newMesh.add_face(capVertices);
                    new_exact_planes[cap_f] = exactPlane;
                } catch (...) {
                    print::error("Failed to add cap face to exact kernel.");
                }
            }
        }
    }

    // Update AABB tracking (move bounds, see paper)
    pmp::Vertex new_aabb_v_min[3];
    pmp::Vertex new_aabb_v_max[3];
    for (int i = 0; i < 3; i++) {
        // Map retained extreme vertices to the new mesh
        if (!min_discarded[i] && state.aabb_v_min[i].is_valid()) {
            new_aabb_v_min[i] = vertexMap[state.aabb_v_min[i]];
        } 
        if (!max_discarded[i] && state.aabb_v_max[i].is_valid()) {
            new_aabb_v_max[i] = vertexMap[state.aabb_v_max[i]]; 
        }
    }

    // Scan the newly generated cut boundary (capVertices) for replacement extremes
    if (!capVertices.empty()) {
        for (int i = 0; i < 3; ++i) {
            if (min_discarded[i]) {
                int64_t best_val = std::numeric_limits<int64_t>::max();
                pmp::Vertex best_ver;
                for (auto cap_v : capVertices) {
                    ExactPoint p = new_exact_points[cap_v];
                    double val = static_cast<double>(p.comp(i)) / static_cast<double>(p.w);
                    int64_t floor = static_cast<int64_t>(std::floor(val));
                    if (floor < best_val) {
                        best_val = floor;
                        best_ver = cap_v;
                    }
                }
                new_aabb_v_min[i] = best_ver;
                state.aabb_min[i] = best_val;
            }
            if (max_discarded[i]) {
                int64_t best_val = std::numeric_limits<int64_t>::lowest();
                pmp::Vertex best_ver;
                for (auto cap_v : capVertices) {
                    ExactPoint p = new_exact_points[cap_v];
                    double val = static_cast<double>(p.comp(i)) / static_cast<double>(p.w);
                    int64_t ceil = static_cast<int64_t>(std::ceil(val));
                    if (ceil > best_val) {
                        best_val = ceil;
                        best_ver = cap_v;
                    }
                }
                new_aabb_v_max[i] = best_ver;
                state.aabb_max[i] = best_val;
            }
        }
    }

    // Save back to state
    for (int i = 0; i < 3; ++i) {
        state.aabb_v_min[i] = new_aabb_v_min[i];
        state.aabb_v_max[i] = new_aabb_v_max[i];
    }
    
    mesh = newMesh;
}

} // namespace mesh_utils
