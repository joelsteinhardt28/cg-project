#include "mesh_utils.hpp"

#include <iostream>
#include <random>
#include <algorithm>

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

    for (auto e : mesh.edges()) {
        if (mesh.is_boundary(e)) continue;

        pmp::Halfedge he0 = mesh.halfedge(e, 0);
        pmp::Halfedge he1 = mesh.halfedge(e, 1);
        pmp::Face f0 = mesh.face(he0);
        pmp::Face f1 = mesh.face(he1);

        // Compute the normal of f0 using the first three vertices of f0
        auto it = mesh.vertices(f0).begin();
        pmp::Point p0 = mesh.position(*it); ++it;
        pmp::Point p1 = mesh.position(*it); ++it;
        pmp::Point p2 = mesh.position(*it);
        pmp::vec3 n_f0 = pmp::cross(p1 - p0, p2 - p0);

        // Find a vertex in f1 that does not share the edge e to test against the plane of f0
        pmp::Vertex opposite;
        for (auto v : mesh.vertices(f1)) {
            if (v != mesh.vertex(e, 0) && v != mesh.vertex(e, 1)) {
                opposite = v;
                break;
            }
        }

        pmp::Point p_test = mesh.position(opposite);
        float det = pmp::dot(n_f0, p_test - p0);  // Scalar triple product (equivalent to 4x4 determinant with hom. coord.)

        // If det > 0, the fourth vertex is above the plane of f0
        if (det > EPSILON) {
            isConcave[f0.idx()] = true;
            isConcave[f1.idx()] = true;
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


/*
 * Perform a cut of the mesh at the given plane. The mesh is required to be convex.
 * The mesh must be loaded in `state.mesh` and the Polyscope visualization will be updated after the cut.
 */
void cut_at_plane(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, bool updateVisuals = false) {
    print::debug("Cut at plane");
    if (mesh.is_empty()) return;

    if (updateVisuals) {
        visualize_cut_plane(state, plane);
    }

    // Lambda helper to discretize vertex states
    auto is_kept = [&](pmp::Point p) {
        return plane.distance(p) <= EPSILON;
    };
    auto is_kept_vertex = [&](pmp::Vertex v) {
        return is_kept(mesh.position(v));
     };

    // Find a edge that crosses the cutting plane to start marching
    pmp::Halfedge start_he = edge_descent(mesh, plane);

    if (!start_he.is_valid()) {
        print::debug("Edge descent failed to find a crossing edge, falling back to linear search.");
        for (auto e : mesh.edges()) {
            if (is_kept_vertex(mesh.vertex(e, 0)) != is_kept_vertex(mesh.vertex(e, 1))) {
                start_he = mesh.halfedge(e, 0);
                break;
            }
        }
    }

    if (!start_he.is_valid()) {
        print::debug("No intersection found with the plane.");

        // Check if all vertices are on the positive side of the plane
        if (mesh.vertices_size() > 0) {
            pmp::Vertex first = *mesh.vertices_begin();
            if (plane.distance(mesh.position(first)) > EPSILON) {
                print::info("All vertices are on the positive side of the plane. Discarding kernel.");
                mesh.clear();
                return;
            }
        }

        return;
    }

    // Orient starting he to point from kept side to discarded side
    if (!is_kept_vertex(mesh.from_vertex(start_he))) {
        start_he = mesh.opposite_halfedge(start_he);
    }

    // March around the intersection loop, recording crossing edges and faces
    std::vector<pmp::Edge> crossingEdges;
    pmp::Halfedge current_he = start_he;

    do {
        print::debug("Current halfedge: " + std::to_string(current_he.idx()));
        pmp::Face current_face = mesh.face(current_he);
        crossingEdges.push_back(mesh.edge(current_he));
        pmp::Halfedge next_he;

        for (auto he : mesh.halfedges(current_face)) {
            if (mesh.edge(he) == mesh.edge(current_he)) continue;  // same edge

            if (is_kept_vertex(mesh.from_vertex(he)) != is_kept_vertex(mesh.to_vertex(he))) {
                next_he = he;
                break;
            }
        }

        if (!next_he.is_valid()) {
            print::error("Marching failed to find next edge.");
            return;
        }

        current_he = mesh.opposite_halfedge(next_he);
    } while (mesh.edge(current_he) != mesh.edge(start_he));

    // Rebuild the mesh
    pmp::SurfaceMesh newMesh;
    std::vector<pmp::Vertex> vertexMap(mesh.n_vertices(), pmp::Vertex());

    for (auto v : mesh.vertices()) {
        if (is_kept_vertex(v)) {
            vertexMap[v.idx()] = newMesh.add_vertex(mesh.position(v));
        }
    }

    std::map<pmp::Edge, pmp::Vertex> edgeIntersections;
    for (auto e : crossingEdges) {
        pmp::Halfedge he = mesh.halfedge(e, 0);
        pmp::Point p0 = mesh.position(mesh.from_vertex(he));
        pmp::Point p1 = mesh.position(mesh.to_vertex(he));
        float d0 = plane.distance(p0);
        float d1 = plane.distance(p1);
        float t = d0 / (d0 - d1);

        edgeIntersections[e] = newMesh.add_vertex(p0 + t * (p1 - p0));
    }

    // Process old faces to generate clipped polygons
    for (auto f : mesh.faces()) {
        std::vector<pmp::Vertex> newFaceVertices;
        bool keptAny = false;

        for (auto he : mesh.halfedges(f)) {
            pmp::Vertex v_from = mesh.from_vertex(he);
            pmp::Vertex v_to = mesh.to_vertex(he);

            if (is_kept_vertex(v_from)) {
                newFaceVertices.push_back(vertexMap[v_from.idx()]);
                keptAny = true;
            }
            if (is_kept_vertex(v_from) != is_kept_vertex(v_to)) {
                newFaceVertices.push_back(edgeIntersections[mesh.edge(he)]);
            }
        }

        if (keptAny && newFaceVertices.size() >= 3) {
            newMesh.add_face(newFaceVertices);
        }
    }

    print::debug("Filling cut hole");
    std::vector<pmp::Vertex> capVertices;
    for (auto e : crossingEdges) {
        capVertices.push_back(edgeIntersections[e]);
    }

    try {
        newMesh.add_face(capVertices);
    } catch (const pmp::TopologyException& e) {
        // If the normal is inverted, try to reverse the sequence
        std::reverse(capVertices.begin(), capVertices.end());
        try {
            newMesh.add_face(capVertices);
        } catch (const pmp::TopologyException& e) {
            print::error("Failed to add cap faces: " + std::string(e.what()));
        }
    }

    mesh = newMesh;
}


/**
 * Cuts the mesh at the given plane, discarding the positive half-space. This algorithm classifies 
 * every vertex of the mesh with respect to its position relative to the plane.
 */
void cut_at_plane_linear_search(AppState& state, pmp::SurfaceMesh& mesh, const Plane& plane, bool updateVisuals = false) {
    print::debug("Perform mesh-plane cutting ...");

    if (mesh.is_empty()) return;

    if (updateVisuals) {
        visualize_cut_plane(state, plane);
    }

    // * TOPOLOGICAL CLASSIFICATION
    // Classify each vertex with respect to the plane using the signed distance.
    // Keep vertices that are in the negative half-space.
    std::vector<bool> keep(mesh.vertices_size(), false);
    for (auto v : mesh.vertices()) {
        keep[v.idx()] = plane.distance(mesh.position(v)) <= EPSILON; 
    }

    // * SEARCH FOR STARTING EDGE
    // Find a starting halfedge for marching. Use Edge Descent bu fall back to Linear 
    // Search if Edge Descent fails. A starting edge is an edge crossing the plane.
    pmp::Halfedge start_he;
    start_he = edge_descent(mesh, plane);

    if (!start_he.is_valid()) {
        print::debug("Edge descent failed to find a crossing edge, falling back to linear search.");
        for (auto e : mesh.edges()) {
            pmp::Halfedge he = mesh.halfedge(e, 0);
            if (keep[mesh.from_vertex(he).idx()] != keep[mesh.to_vertex(he).idx()]) {
                start_he = he;
                break;
            }
        }
    }

    if (!start_he.is_valid()) {
        print::warning("No intersection found with the plane.");
        return;
    }

    // * MARCHING
    // Walk along the intersection using topological classification and record
    // crossing edges and faces until we return to the starting edge.
    std::vector<pmp::Edge> crossingEdges;
    std::vector<pmp::Face> crossingFaces;
    pmp::Halfedge current_he = start_he;

    do {
        // current_he = start_he, so we know that it crosses the plane
        pmp::Face current_face = mesh.face(current_he);
        crossingEdges.push_back(mesh.edge(current_he));
        crossingFaces.push_back(current_face);

        pmp::Halfedge next_he;
        for (auto he : mesh.halfedges(current_face)) {
            if (mesh.edge(he) == mesh.edge(current_he)) continue;  

            pmp::Vertex v0 = mesh.from_vertex(he);
            pmp::Vertex v1 = mesh.to_vertex(he);

            if (keep[v0.idx()] != keep[v1.idx()]) {
                next_he = he;
                break;
            }
        }

        if (!next_he.is_valid()) {
            print::warning("Marching failed to find next edge.");
            return;
        }

        current_he = mesh.opposite_halfedge(next_he);
    } while (mesh.edge(current_he) != mesh.edge(start_he));

    // * SPLIT CROSSING EDGES
    // For each crossing edge, split it at the intersection point and keep track of the new vertex.
    std::vector<pmp::Vertex> newVertices;
    for (const auto& e : crossingEdges) {
        pmp::Halfedge he = mesh.halfedge(e, 0);
        pmp::Point p0 = mesh.position(mesh.from_vertex(he));
        pmp::Point p1 = mesh.position(mesh.to_vertex(he));
        float d0 = plane.distance(p0);
        float d1 = plane.distance(p1);
        
        float t = d0 / (d0 - d1);
        
        // Clamp t to prevent zero-length edges (never cut exactly at a vertex)
        t = std::max(0.001f, std::min(0.999f, t)); 
        
        pmp::Point newPos = p0 + t * (p1 - p0); 
        pmp::Halfedge new_he = mesh.split(e, newPos);
        newVertices.push_back(mesh.to_vertex(new_he));
    }

    // // * SPLIT CROSSING FACES 
    // // Connect the new vertices in a loop to form the cut edge.
    // print::debug("We have in total " + std::to_string(crossingFaces.size()) + " crossing faces to split.");
    // for (size_t i = 0; i < crossingFaces.size(); ++i) {
    //     print::debug("» Attempting to split face " + std::to_string(crossingFaces[i].idx()));
    //     pmp::Face face = crossingFaces[i];
    //     pmp::Vertex v1 = newVertices[i];
    //     pmp::Vertex v2 = newVertices[(i + 1) % newVertices.size()];

    //     pmp::Halfedge first_he, second_he;
    //     // for (auto he : mesh.halfedges(face)) {
    //     //     if (mesh.to_vertex(he) == v1) {  // ! removed mesh.to_vertex(he) == v2
    //     //         if (!first_he.is_valid()) {
    //     //             first_he = he;
    //     //         } else {
    //     //             second_he = he;
    //     //             break;
    //     //         }
    //     //     } 
    //     // }
    //     print::debug("Total halfedges in face " + std::to_string(face.idx()) + ": " + std::to_string(mesh.valence(face)));
    //     for (auto he : mesh.halfedges(face)) {
    //         if (mesh.from_vertex(he) == v1) first_he = he;
    //         if (mesh.from_vertex(he) == v2) second_he = he;
    //         print::debug("Checking halfedge " + std::to_string(he.idx()) + " with from_vertex " + std::to_string(mesh.from_vertex(he).idx()));
    //     }

    //     if (first_he.is_valid() && second_he.is_valid()) {
    //         mesh.insert_edge(first_he, second_he);
    //     } else {
    //         print::error("Failed to find halfedges for face splitting.");
    //     }
    // }

    // * DELETE POSITIVE VERTICES
    std::vector<pmp::Vertex> toDelete;
    for (auto v : mesh.vertices()) {
        // Only evaluate original vertices using the keep array bounds
        if (v.idx() < keep.size() && !keep[v.idx()]) {
            toDelete.push_back(v);
        }
    }
    for (auto v : toDelete) {
        mesh.delete_vertex(v);
    }
    mesh.garbage_collection();

    // * FILL HOLE & TRIANGULATE
    std::vector<pmp::Vertex> newFaceVertices;
    pmp::Halfedge boundary_start;
    for (auto he : mesh.halfedges()) {
        if (mesh.is_boundary(he)) {
            boundary_start = he;
            break;
        }
    }

    if (boundary_start.is_valid()) {
        pmp::Halfedge current_he = boundary_start;
        do {
            newFaceVertices.push_back(mesh.to_vertex(current_he));
            current_he = mesh.next_halfedge(current_he);
        } while (current_he != boundary_start);

        try {
            pmp::Face new_face = mesh.add_face(newFaceVertices);
            pmp::triangulate(mesh, new_face);
        } catch (const pmp::TopologyException& e) {
            std::reverse(newFaceVertices.begin(), newFaceVertices.end());
            try {
                pmp::Face new_face = mesh.add_face(newFaceVertices);
                pmp::triangulate(mesh, new_face);
            } catch (const pmp::TopologyException& e) {
                print::warning("Failed to fill cut hole: " + std::string(e.what()));
            }
        }
    }

    pmp::triangulate(mesh);
}

} // namespace mesh_utils
