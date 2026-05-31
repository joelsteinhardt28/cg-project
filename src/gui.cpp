#include "gui.hpp"

#include <filesystem>

#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "imgui.h"

#include <pmp/surface_mesh.h>
#include <pmp/io/io.h>

#include "io.hpp"
#include "mesh_utils.hpp"

namespace gui {

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

                    state.sc = mesh_utils::registerPmpMesh("Mesh", state.mesh);
                    state.pc = mesh_utils::registerPmpPointCloud("Points", state.mesh);
                    if (!state.mesh.is_empty()) state.meshLoaded = true;

                    mesh_utils::registerBoundingBox(state);

                    // Point cloud cosmetics
                    state.pc->setEnabled(false);

                    // Reset camera to fit the new mesh
                    polyscope::view::resetCameraToHomeView();
                } catch (const std::exception& e) {
                    std::cerr << "[ERROR] Error while loading " << selectedPath << ": " << e.what() << std::endl;
                    polyscope::error("Failed to load mesh: " + std::string(e.what()));
                    state.selectedOffFileIdx = -1;
                    state.meshLoaded = false;
                }
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }


    if (state.meshLoaded) {
        if (ImGui::Button("Generate Random Cutting Plane")) {
            mesh_utils::generate_random_bbox_plane(state);
        }

        if (state.hasActiveCutPlane) {
            ImGui::SameLine();
            if (ImGui::Button("Cut")) {
                mesh_utils::cut_at_plane(state, state.activeCutPlane);
                polyscope::removeSurfaceMesh("Clipped Random Plane");
                state.hasActiveCutPlane = false;
            }
        }
        
    }
}

} // namespace gui
