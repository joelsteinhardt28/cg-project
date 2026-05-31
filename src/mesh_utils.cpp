#include "mesh_utils.hpp"

#include <iostream>
#include <random>
#include <algorithm>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

#include <pmp/surface_mesh.h>
#include <pmp/bounding_box.h>
#include <pmp/algorithms/utilities.h>
#include <pmp/exceptions.h>

namespace mesh_utils {

polyscope::SurfaceMesh* registerPmpMesh(const std::string& name, const pmp::SurfaceMesh& mesh) {
    std::cout << "[INFO] Registering PMP mesh with Polyscope: " << name << std::endl;
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

polyscope::PointCloud* registerPmpPointCloud(const std::string& name, const pmp::SurfaceMesh& mesh) {
    std::cout << "[INFO] Registering PMP point cloud with Polyscope: " << name << std::endl;
    std::vector<Point> vertices;
    vertices.reserve(mesh.n_vertices());
    for (auto v : mesh.vertices()) {
        auto p = mesh.position(v);
        vertices.push_back(p);
    }
    return polyscope::registerPointCloud(name, vertices);
}

void registerBoundingBox(AppState& state) {
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
    auto* bboxCN = polyscope::registerCurveNetwork("Bounding Box", state.bboxVertices, bboxEdges);
    bboxCN->setRadius(0.001);
    bboxCN->setColor({0.4, 0.4, 0.4});
}

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

    auto* planeMesh = polyscope::registerSurfaceMesh("Clipped Random Plane", polygon, faces);
    planeMesh->setSurfaceColor({0.8f, 0.1f, 0.2f});
    planeMesh->setTransparency(0.6f);
}

pmp::Halfedge edge_descent(pmp::SurfaceMesh& mesh, const Plane& plane) {
    if (mesh.is_empty()) return pmp::Halfedge();

    pmp::Vertex current_v = *mesh.vertices_begin();
    float current_dist = plane.distance(mesh.position(current_v));

    while(true) {
        pmp::Vertex bestNeighbor;
        float min_abs_dist = std::abs(current_dist);
        bool foundCloser = false;

        for (auto he : mesh.halfedges(current_v)) {
            pmp::Vertex neighbor = mesh.to_vertex(he);
            float neighbor_dist = plane.distance(mesh.position(neighbor));

            if ((current_dist > EPSILON && neighbor_dist < -EPSILON) ||
                (current_dist < -EPSILON && neighbor_dist > EPSILON)) {
                return he;
            }

            if (std::abs(neighbor_dist) < min_abs_dist) {
                min_abs_dist = std::abs(neighbor_dist);
                bestNeighbor = neighbor;
                foundCloser = true;
            }
        }

        if (!foundCloser) return pmp::Halfedge();

        current_v = bestNeighbor;
        current_dist = plane.distance(mesh.position(current_v));
    }
}

void cut_at_plane(AppState& state, const Plane& plane) {
    if (!state.meshLoaded || state.mesh.is_empty()) return;

    pmp::Halfedge start_he = edge_descent(state.mesh, plane);
    if (!start_he.is_valid()) {
        std::cout << "[INFO] No intersection found with the plane." << std::endl;
        return;
    }

    std::vector<pmp::Edge> crossingEdges;
    std::vector<pmp::Face> crossingFaces;
    pmp::Halfedge current_he = start_he;

    do {
        pmp::Face current_face = state.mesh.face(current_he);
        crossingEdges.push_back(state.mesh.edge(current_he));
        crossingFaces.push_back(current_face);

        pmp::Halfedge next_he;
        for (auto he : state.mesh.halfedges(current_face)) {
            if (state.mesh.edge(he) == state.mesh.edge(current_he)) continue;

            pmp::Point p0 = state.mesh.position(state.mesh.from_vertex(he));
            pmp::Point p1 = state.mesh.position(state.mesh.to_vertex(he));
            float d0 = plane.distance(p0);
            float d1 = plane.distance(p1);

            if ((d0 > EPSILON && d1 < -EPSILON) || (d0 < -EPSILON && d1 > EPSILON)) {
                next_he = he;
                break;
            }
        }

        if (!next_he.is_valid()) {
            std::cerr << "[ERROR] Marching failed to find next edge." << std::endl;
            return;
        }

        current_he = state.mesh.opposite_halfedge(next_he);
    } while (state.mesh.edge(current_he) != state.mesh.edge(start_he));

    std::vector<pmp::Vertex> newVertices;
    for (const auto& e : crossingEdges) {
        pmp::Halfedge he = state.mesh.halfedge(e, 0);
        pmp::Point p0 = state.mesh.position(state.mesh.from_vertex(he));
        pmp::Point p1 = state.mesh.position(state.mesh.to_vertex(he));
        float d0 = plane.distance(p0);
        float d1 = plane.distance(p1);
        float t = d0 / (d0 - d1);
        pmp::Point newPos = p0 + t * (p1 - p0);

        pmp::Halfedge new_he = state.mesh.split(e, newPos);
        pmp::Vertex new_v = state.mesh.to_vertex(new_he);
        newVertices.push_back(new_v);
    }

    for (size_t i = 0; i < crossingFaces.size(); ++i) {
        pmp::Face f = crossingFaces[i];
        pmp::Vertex v1 = newVertices[i];
        pmp::Vertex v2 = newVertices[(i + 1) % newVertices.size()];

        pmp::Halfedge he0, he1;
        for (auto he : state.mesh.halfedges(f)) {
            if (state.mesh.to_vertex(he) == v1) he0 = he;
            if (state.mesh.to_vertex(he) == v2) he1 = he;
        }

        if (he0.is_valid() && he1.is_valid()) {
            state.mesh.insert_edge(he0, he1);
        }
    }

    std::vector<pmp::Vertex> toDelete;
    for (auto v : state.mesh.vertices()) {
        if (plane.distance(state.mesh.position(v)) > EPSILON) {
            toDelete.push_back(v);
        }
    }
    for (auto v : toDelete) {
        state.mesh.delete_vertex(v);
    }
    state.mesh.garbage_collection();

    std::vector<pmp::Vertex> newFaceVertices;
    pmp::Halfedge boundary_start;
    for (auto he : state.mesh.halfedges()) {
        if (state.mesh.is_boundary(he)) {
            boundary_start = he;
            break;
        }
    }

    if (boundary_start.is_valid()) {
        pmp::Halfedge current_he = boundary_start;
        do {
            newFaceVertices.push_back(state.mesh.to_vertex(current_he));
            current_he = state.mesh.next_halfedge(current_he);
        } while (current_he != boundary_start);

        try {
            state.mesh.add_face(newFaceVertices);
        } catch (const pmp::TopologyException& e) {
            std::reverse(newFaceVertices.begin(), newFaceVertices.end());
            try {
                state.mesh.add_face(newFaceVertices);
            } catch (const pmp::TopologyException& e) {
                std::cerr << "[ERROR] Failed to fill cut hole: " << e.what() << std::endl;
            }
        }
    }

    polyscope::removeSurfaceMesh("Mesh");
    state.sc = registerPmpMesh("Mesh", state.mesh);
    registerBoundingBox(state);
}

} // namespace mesh_utils
