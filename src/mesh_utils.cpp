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

/**
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

/**
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

/**
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


/**
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


/**
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

/*
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


pmp::Halfedge edge_descent(pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane) {
    print::info("Edge descent with exact plane classification");
    if (mesh.is_empty()) return pmp::Halfedge();

    auto exact_points = mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    if (!exact_points) return pmp::Halfedge();

    pmp::Vertex current_v = *mesh.vertices_begin();  // start from an arbitrary vertex
    float current_dist = plane.distance(mesh.position(current_v));
    std::vector<bool> visited(mesh.n_vertices(), false);

    auto exact_neighborhood_check = [&](pmp::Vertex v) -> pmp::Halfedge {
        int c0 = ipg::classify(exact_points[v], exactPlane);

        // If the vertex is on the plane, a strict crossing edge does not exist
        if (c0 == 0) return pmp::Halfedge();

        for (auto he : mesh.halfedges(v)) {
            pmp::Vertex neighbor = mesh.to_vertex(he);
            int c1 = ipg::classify(exact_points[neighbor], exactPlane);

            if (c1 == 0) return pmp::Halfedge();

            if ((c0 == -1 && c1 == 1) || (c0 == 1 && c1 == -1)) {
                return he; // Found edge crossing the plane
            }
        }
        return pmp::Halfedge();
    };

    while(current_v.is_valid()) {
        visited[current_v.idx()] = true;

        // Dynamic epsilon: If we are very close to the plane, we switch to integer check
        float max_coord = std::max({
            std::abs(mesh.position(current_v)[0]),
            std::abs(mesh.position(current_v)[1]),
            std::abs(mesh.position(current_v)[2])
        });
        float epsilon = std::abs(max_coord - std::nextafter(max_coord, std::numeric_limits<float>::infinity())) * 2.0f;
        if (epsilon > EPSILON) {
            epsilon = EPSILON;
        }

        if (std::abs(current_dist) <= epsilon) {
            return exact_neighborhood_check(current_v);
        }

        pmp::Vertex bestNeighbor;
        float min_abs_dist = std::abs(current_dist);
        bool foundCloser = false;

        for (auto he : mesh.halfedges(current_v)) {
            pmp::Vertex neighbor = mesh.to_vertex(he);

            // Look for direction which brings us closer to the plane
            if (!visited[neighbor.idx()]) {
                float neighbor_dist = plane.distance(mesh.position(neighbor));

                if ((current_dist > 0 && neighbor_dist < 0) || (current_dist < 0 && neighbor_dist > 0)) {
                    pmp::Halfedge crossing_he = exact_neighborhood_check(current_v);
                    if (crossing_he.is_valid()) return crossing_he; // Found edge crossing the plane
                }

                if (std::abs(neighbor_dist) < min_abs_dist) {
                    min_abs_dist = std::abs(neighbor_dist);
                    bestNeighbor = neighbor;
                    foundCloser = true;
                }
            }
        }

        // No vertex brings us closer to the plane, so we are at a local minimum
        if (!foundCloser) {
            // Final check
            pmp::Halfedge final_check = exact_neighborhood_check(current_v);
            if (final_check.is_valid()) return final_check;

            print::warning("Edge descent hit a local minimum.");
            return pmp::Halfedge();
        }

        current_v = bestNeighbor;
        current_dist = plane.distance(mesh.position(current_v));
    }

    return pmp::Halfedge(); // No crossing edge found
}


/**
 * Perform a cut of the mesh at the given plane. The mesh is required to be convex.
 * The mesh must be loaded in `state.mesh` and the Polyscope visualization will be updated after the cut.
 */
void cut_at_plane(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane, bool updateVisuals) {
    print::debug("Cut at plane (exact with state machine)");
    if (mesh.is_empty()) return;

    if (updateVisuals) visualize_cut_plane(state, plane);

    auto exact_points = mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    auto exact_planes = mesh.get_face_property<ExactPlane>("f:exact_plane");

    // * Exact vertex classification (-1: Keep, 0: On plane, 1: Discard)
    auto get_v_class = [&](pmp::Vertex v) -> int {
        if (exact_points) {
            return ipg::classify(exact_points[v], exactPlane);
        } else {
            float dist = plane.distance(mesh.position(v));
            if (std::abs(dist) < EPSILON) return 0;
            return (dist < 0) ? -1 : 1;
        }
    };

    // Find a edge that crosses the cutting plane to start marching
    pmp::Halfedge start_he = edge_descent(mesh, plane, exactPlane);

    if (!start_he.is_valid()) {
        print::debug("Edge descent failed to find a crossing edge, falling back to linear search.");
        for (auto e : mesh.edges()) {
            int c0 = get_v_class(mesh.vertex(e, 0));
            int c1 = get_v_class(mesh.vertex(e, 1));
            if ((c0 == -1 && c1 == 1) || (c0 == 1 && c1 == -1)) {
                start_he = mesh.halfedge(e, 0);
                break;
            }
        }
    }

    if (!start_he.is_valid()) {
        print::debug("No intersection found with the plane.");

        // Check if all vertices are on the positive side of the plane
        if (mesh.vertices_size() > 0 && get_v_class(*mesh.vertices_begin()) <= 0) {
            return; // If the first vertex is kept, all are kept
        } else {
            print::info("All vertices are on the positive side of the plane. Discarding kernel.");
            mesh.clear();
            return;
        }

        return;
    }

    // Orient starting he to point from kept side (-1) to discarded side (1)
    if (get_v_class(mesh.from_vertex(start_he)) == 1) {
        start_he = mesh.opposite_halfedge(start_he);
    }

    // March around the intersection loop
    struct CutPoint {
        bool isEdge;
        pmp::Edge e;
        pmp::Vertex v;
    };
    std::vector<CutPoint> cutLoop;
    pmp::Halfedge current_he = start_he;

    do {
        print::debug("Current halfedge: " + std::to_string(current_he.idx()));

        // Record the entry edge (which always goes from -1 to 1)
        cutLoop.push_back({true, mesh.edge(current_he), pmp::Vertex()});

        pmp::Face current_face = mesh.face(current_he);

        // Trace the perimeter of the polygon to find the exit
        pmp::Halfedge h = mesh.next_halfedge(current_he);
        pmp::Halfedge exit_edge;
        pmp::Vertex exit_vertex;

        while (h != current_he) {
            int c_to = get_v_class(mesh.to_vertex(h));
            
            if (c_to == -1) {
                // The perimeter transitioned from Discarded (1) back to Kept (-1).
                // This is our exit edge!
                exit_edge = h;
                break;
            } else if (c_to == 0) {
                // We hit a vertex lying exactly on the cut plane.
                exit_vertex = mesh.to_vertex(h);
                break;
            }
            h = mesh.next_halfedge(h);
        }

        if (exit_edge.is_valid()) {
            // Clean edge exit: Step across the edge into the adjacent face
            // Because exit_edge goes from 1 to -1, the opposite goes from -1 to 1!
            current_he = mesh.opposite_halfedge(exit_edge); 
            
        } else if (exit_vertex.is_valid()) {
            // Degenerate vertex exit
            cutLoop.push_back({false, pmp::Edge(), exit_vertex});

            bool foundNextEntry = false;
            
            // Pivot around the vertex to find which adjacent face the plane continues into
            for (auto h_ring : mesh.halfedges(exit_vertex)) {
                pmp::Face f_pivot = mesh.face(h_ring);
                
                // Ignore invalid faces and the face we just came from
                if (!f_pivot.is_valid() || f_pivot == current_face) continue;

                // Search this adjacent polygon for an entry edge (-1 to 1)
                for (auto h_f : mesh.halfedges(f_pivot)) {
                    if (get_v_class(mesh.from_vertex(h_f)) == -1 && 
                        get_v_class(mesh.to_vertex(h_f)) == 1) {
                        current_he = h_f;
                        foundNextEntry = true;
                        break;
                    }
                }
                if (foundNextEntry) break;
            }

            if (!foundNextEntry) {
                print::error("Marching failed to find next edge after hitting a vertex.");
                return;
            }
        } else {
            print::error("Failed to find any exit from the current face.");
            return;
        }

    } while (mesh.edge(current_he) != mesh.edge(start_he));

    // Rebuild the mesh
    pmp::SurfaceMesh newMesh;
    auto new_exact_points = newMesh.add_vertex_property<ExactPoint>("v:exact_pos");
    auto new_exact_planes = newMesh.add_face_property<ExactPlane>("f:exact_plane");

    std::map<pmp::Vertex, pmp::Vertex> vertexMap;

    for (auto v : mesh.vertices()) {
        if (get_v_class(v) <= 0) {
            auto nv = newMesh.add_vertex(mesh.position(v));
            vertexMap[v] = nv;
            if (exact_points) new_exact_points[nv] = exact_points[v];
        }
    }

    // Compute exact plane-plane-plane intersections
    std::map<pmp::Edge, pmp::Vertex> edgeIntersections;
    std::vector<pmp::Vertex> capVertices;

    for (const auto& cp : cutLoop) {
        if (cp.isEdge) {
            // Only compute true intersections if we haven't already
            if (edgeIntersections.find(cp.e) == edgeIntersections.end()) {
                pmp::Face f0 = mesh.face(mesh.halfedge(cp.e, 0));
                pmp::Face f1 = mesh.face(mesh.halfedge(cp.e, 1));

                ExactPoint pt = ipg::intersect(exact_planes[f0], exact_planes[f1], exactPlane);

                tg::pos3 tg_pos = ipg::to_pos3_fast(pt);
                pmp::Point float_pos(tg_pos.x, tg_pos.y, tg_pos.z);
                float_pos /= static_cast<float>(globalSettings::scaleFactor);

                auto nv = newMesh.add_vertex(float_pos);
                if (exact_points) new_exact_points[nv] = pt;
                edgeIntersections[cp.e] = nv;
            }

            // Preserve the topological ordering of the cap face
            capVertices.push_back(edgeIntersections[cp.e]);
        } else {
            // Degenerate case: the cut passes through a vertex
            capVertices.push_back(vertexMap[cp.v]);
        }
    }

    // Rebuild polygon faces
    for (auto f : mesh.faces()) {
        std::vector<pmp::Vertex> newFaceVertices;

        for (auto he : mesh.halfedges(f)) {
            pmp::Vertex v_from = mesh.from_vertex(he);
            pmp::Vertex v_to = mesh.to_vertex(he);

            if (get_v_class(v_from) <= 0) {
                newFaceVertices.push_back(vertexMap[v_from]);
            }
            if ((get_v_class(v_from) < 0 && get_v_class(v_to) > 0) ||
                (get_v_class(v_from) > 0 && get_v_class(v_to) < 0)) {
                newFaceVertices.push_back(edgeIntersections[mesh.edge(he)]);
            }
        }

        if (newFaceVertices.size() >= 3) {
            try {
                auto nf = newMesh.add_face(newFaceVertices);
                if (exact_planes) new_exact_planes[nf] = exact_planes[f];
            } catch (const pmp::TopologyException& e) {
                print::error("Failed to add face: " + std::string(e.what()));
            }
        }
    }

    // Fill the cut hole with a new face
    print::debug("Filling cut hole");
    if (capVertices.size() >= 3) {
        // Clean up any double-adds from pivoting
        capVertices.erase(std::unique(capVertices.begin(), capVertices.end()), capVertices.end());

        if (capVertices.size() >= 3) {
            try {
                auto capFace = newMesh.add_face(capVertices);
                if (exact_planes) new_exact_planes[capFace] = exactPlane;
            } catch (const pmp::TopologyException& e) {
                std::reverse(capVertices.begin(), capVertices.end());
                try {
                    auto capFace = newMesh.add_face(capVertices);
                    if (exact_planes) new_exact_planes[capFace] = exactPlane;
                } catch (const pmp::TopologyException& e) {
                    print::error("Failed to add cap face: " + std::string(e.what()));
                }
            }
        }
    }

    mesh = std::move(newMesh);
}

void cut_at_plane(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, bool updateVisuals) {
    pmp::vec3 n = pmp::normalize(plane.normal);
    pmp::vec3 u, v;
    pmp::vec3 perp;
    if (std::abs(n[0]) > 0.9f) {
        perp = pmp::vec3(0.0f, 1.0f, 0.0f);
    } else {
        perp = pmp::vec3(1.0f, 0.0f, 0.0f);
    }
    u = pmp::normalize(pmp::cross(n, perp));
    v = pmp::normalize(pmp::cross(n, u));

    pmp::Point p0 = -plane.d * n;
    pmp::Point p1 = p0 + u;
    pmp::Point p2 = p0 + v;

    auto to_ipos = [](pmp::Point p) {
        return tg::ipos3(
            static_cast<int64_t>(p[0] * globalSettings::scaleFactor),
            static_cast<int64_t>(p[1] * globalSettings::scaleFactor),
            static_cast<int64_t>(p[2] * globalSettings::scaleFactor)
        );
    };

    tg::pos<3, ExactGeom::pos_scalar_t> ip0(to_ipos(p0));
    tg::pos<3, ExactGeom::pos_scalar_t> ip1(to_ipos(p1));
    tg::pos<3, ExactGeom::pos_scalar_t> ip2(to_ipos(p2));

    ExactPlane exactPlane = ExactPlane::from_points(ip0, ip1, ip2);
    cut_at_plane(state, mesh, plane, exactPlane, updateVisuals);
}


void cut_at_plane_linear(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, const ExactPlane& exactPlane, bool updateVisuals) {
    if (mesh.is_empty()) return;
    if (updateVisuals) visualize_cut_plane(state, plane);

    auto exact_points = mesh.get_vertex_property<ExactPoint>("v:exact_pos");
    auto exact_planes = mesh.get_face_property<ExactPlane>("f:exact_plane");

    // 1. Exact Vertex Classification (-1: Keep, 0: On Plane, 1: Discard)
    std::map<pmp::Vertex, int> v_class;
    bool all_positive = true, all_negative = true;
    for (auto v : mesh.vertices()) {
        v_class[v] = ipg::classify(exact_points[v], exactPlane);
        if (v_class[v] <= 0) all_positive = false; 
        if (v_class[v] > 0) all_negative = false;  
    }

    if (all_positive) {
        mesh.clear(); // Entire kernel was discarded
        return;
    }
    if (all_negative) return; // Cut plane missed the kernel completely

    pmp::SurfaceMesh newMesh;
    auto new_exact_points = newMesh.add_vertex_property<ExactPoint>("v:exact_pos");
    auto new_exact_planes = newMesh.add_face_property<ExactPlane>("f:exact_plane");

    std::map<pmp::Vertex, pmp::Vertex> vertexMap;
    std::map<pmp::Edge, pmp::Vertex> edgeIntersections;
    std::vector<pmp::Vertex> capVertices;

    // 2. Map kept vertices (and track ones exactly on the plane)
    for (auto v : mesh.vertices()) {
        if (v_class[v] <= 0) {
            auto nv = newMesh.add_vertex(mesh.position(v));
            vertexMap[v] = nv;
            new_exact_points[nv] = exact_points[v];
            if (v_class[v] == 0) capVertices.push_back(nv); 
        }
    }

    // 3. Compute EXACT intersections for strictly crossing edges
    for (auto e : mesh.edges()) {
        pmp::Vertex v0 = mesh.vertex(e, 0);
        pmp::Vertex v1 = mesh.vertex(e, 1);
        
        // Only split if strictly crossing (-1 to 1)
        if ((v_class[v0] < 0 && v_class[v1] > 0) || (v_class[v0] > 0 && v_class[v1] < 0)) {
            pmp::Face f0 = mesh.face(mesh.halfedge(e, 0));
            pmp::Face f1 = mesh.face(mesh.halfedge(e, 1));
            
            ExactPlane p0 = exact_planes[f0];
            ExactPlane p1 = exact_planes[f1];
            
            // True Homogeneous Plane-Plane-Plane Intersection
            ExactPoint pt = ipg::intersect(p0, p1, exactPlane);
            
            tg::pos3 tg_pos = ipg::to_pos3_fast(pt);
            pmp::Point float_pos(tg_pos.x, tg_pos.y, tg_pos.z);
            float_pos /= static_cast<float>(globalSettings::scaleFactor);
            
            auto nv = newMesh.add_vertex(float_pos);
            new_exact_points[nv] = pt;
            edgeIntersections[e] = nv;
            capVertices.push_back(nv);
        }
    }

    // 4. Rebuild the clipped faces
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
                    new_exact_planes[nf] = exact_planes[f]; // Preserve the exact support plane
                } catch (...) {} // Ignore silently degenerate faces at corners
            }
        }
    }

    // 5. Fill the cap face
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
                new_exact_planes[cap_f] = exactPlane; // The cut plane IS the new exact plane
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
    
    mesh = newMesh;
}



} // namespace mesh_utils
