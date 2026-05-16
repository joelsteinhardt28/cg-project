#include "gui.hpp"

#include <filesystem>

#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/pick.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "portable-file-dialogs.h"
#include "imgui.h"

#include <pmp/surface_mesh.h>
#include <pmp/io/io.h>
#include <pmp/bounding_box.h>
#include <pmp/algorithms/utilities.h>
#include <pmp/exceptions.h>

#include "io.hpp"

namespace gui {

polyscope::SurfaceMesh* registerPmpMesh(const std::string& name, const pmp::SurfaceMesh& mesh) {
    std::cout << "[INFO] Registering PMP mesh with Polyscope: " << name << std::endl;
    std::vector<Point> vertices;
    vertices.reserve(mesh.n_vertices());
    for (auto v : mesh.vertices()) {
        auto p = mesh.position(v);
        vertices.push_back({p[0], p[1], p[2]});
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
        vertices.push_back({p[0], p[1], p[2]});
    }
    return polyscope::registerPointCloud(name, vertices);
}

void generate_random_cutting_plane(AppState& state) {
    polyscope::removeAllSlicePlanes();

    const auto& points = state.bboxVertices.empty() ? std::vector<Point>() : state.bboxVertices;
    if (points.size() < 3) return;
    
    // Pick 3 random points to define a valid plane in the cloud
    Point p1 = points[rand() % points.size()];
    Point p2 = points[rand() % points.size()];
    Point p3 = points[rand() % points.size()];
    
    // Compute vectors between points
    Point v1 = {p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]};
    Point v2 = {p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]};
    
    // Compute normal
    glm::vec3 normal(
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0]
    );
    normal = glm::normalize(normal);
    
    // Center of the plane (we can just use p1)
    glm::vec3 center(p1[0], p1[1], p1[2]);

    // Add a Polyscope slice plane
    polyscope::SlicePlane* slice = polyscope::addSceneSlicePlane("Random Slice");
    slice->setPose(center, normal);
    slice->setDrawPlane(true);
    slice->setDrawWidget(true);
}

/*
Generates a random cutting plane through the bbox of the mesh, clipped to the bbox.
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

    // Generate a large quad for the plane, then we will clip it to the bbox. 
    // We can use the same tangent/bitangent generation as in the picking code to get a stable orientation for the plane.
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

    // Visualize the clipped polygon
    // Polyscope handles N-sided polygons, so we just pass the vertices as one face
    std::vector<std::vector<size_t>> faces;
    std::vector<size_t> face;
    for(size_t i=0; i<polygon.size(); ++i) face.push_back(i);
    faces.push_back(face);

    auto* planeMesh = polyscope::registerSurfaceMesh("Clipped Random Plane", polygon, faces);
    planeMesh->setSurfaceColor({0.8f, 0.1f, 0.2f});
    planeMesh->setTransparency(0.6f);
}


// * Helper to scan the OFF file directory and populate the list in AppState of available files
void refreshOffFileList(AppState& state) {
    state.offFiles.clear();
    if (!std::filesystem::exists(state.targetDir)) return;

    for (const auto& entry : std::filesystem::directory_iterator(state.targetDir)) {
        if (entry.path().extension() == ".off") {
            state.offFiles.push_back(entry.path().filename().string());
        }
    }
}    

void render(AppState& state) {
    if (state.offFiles.empty()) {
        refreshOffFileList(state);
    }

    // * Dropdown to select OFF file
    const char* preview = (state.selectedOffFileIdx >= 0) ? state.offFiles[state.selectedOffFileIdx].c_str() : "Select an OFF file";

    if (ImGui::BeginCombo("OFF File", preview)) {
        int nFiles = state.offFiles.size();
        for (int i = 0; i < nFiles; i++) {
            const bool isSelected = (state.selectedOffFileIdx == i);
            if (ImGui::Selectable(state.offFiles[i].c_str(), isSelected)) {
                state.selectedOffFileIdx = i;
                std::filesystem::path selectedPath = std::filesystem::path(state.targetDir) / state.offFiles[i];
                
                try {
                    pmp::SurfaceMesh tempMesh;
                    std::cout << "[INFO] Reading mesh from: " << selectedPath << std::endl;
                    pmp::read(tempMesh, selectedPath);
                    state.mesh = std::move(tempMesh);
                    std::cout << "[INFO] Mesh loaded: " << state.mesh.n_vertices() << " vertices, " << state.mesh.n_faces() << " faces." << std::endl;

                    state.sc = registerPmpMesh("Mesh", state.mesh);
                    state.pc = registerPmpPointCloud("Points", state.mesh);
                    if (!state.mesh.is_empty()) state.meshLoaded = true;

                    registerBoundingBox(state);

                    // Point cloud cosmetics
                    state.pc->setEnabled(false);

                    // Reset camera to fit the new mesh
                    polyscope::view::resetCameraToHomeView();
                } catch (const pmp::TopologyException& e) {
                    std::cerr << "[ERROR] Topology error while loading " << selectedPath << ": " << e.what() << std::endl;
                    polyscope::error("Failed to load mesh: " + std::string(e.what()) + "\nThis is likely due to non-manifold geometry.");
                    state.selectedOffFileIdx = -1;
                    state.meshLoaded = false;
                } catch (const std::exception& e) {
                    std::cerr << "[ERROR] Error while loading " << selectedPath << ": " << e.what() << std::endl;
                    polyscope::error("Failed to load mesh: " + std::string(e.what()));
                    state.selectedOffFileIdx = -1;
                    state.meshLoaded = false;
                }
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }


    if (state.meshLoaded) {
        // Show these options only if the spatial data structure actually exists (i.e. after building)

        if (ImGui::Button("Generate Random Cutting Plane")) {
            // generate_random_cutting_plane(state);
            generate_random_bbox_plane(state);
        }
        
    }
}

void registerBoundingBox(AppState& state) {
    if (!state.meshLoaded) return;

    pmp::BoundingBox bbox = pmp::bounds(state.mesh);

    state.bboxVertices = {
        {bbox.min()[0], bbox.min()[1], bbox.min()[2]}, // 0
        {bbox.max()[0], bbox.min()[1], bbox.min()[2]}, // 1
        {bbox.max()[0], bbox.max()[1], bbox.min()[2]}, // 2
        {bbox.min()[0], bbox.max()[1], bbox.min()[2]}, // 3
        {bbox.min()[0], bbox.min()[1], bbox.max()[2]}, // 4
        {bbox.max()[0], bbox.min()[1], bbox.max()[2]}, // 5
        {bbox.max()[0], bbox.max()[1], bbox.max()[2]}, // 6
        {bbox.min()[0], bbox.max()[1], bbox.max()[2]}  // 7
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

} // namespace gui
