#include "gui.hpp"

#include <filesystem>

#include "polyscope/messages.h"
#include "imgui.h"

#include <pmp/io/io.h>

#include "io.hpp"
#include "mesh_utils.hpp"
#include "kernel_gen.hpp"

namespace gui {

/**
 * Resets the polyscope application state by clearing all registered structures and resetting the AppState
 * fields to their default values.
 */
void reset(AppState& state) {
    print::info("Resetting application state...");
    polyscope::removeAllStructures();
    state.mesh = pmp::SurfaceMesh();
    state.meshLoaded = false;
    state.oSMesh = nullptr;
    state.pc = nullptr;
    state.kSMesh = nullptr;
    state.supportPlanes.clear();
    state.isSteppingKernel = false;
    state.currentPlaneIdx = 0;
}

/**
 * Helper to scan the OFF file directory and populate the list in AppState of available files
 */
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
                reset(state);
                state.selectedOffFileIdx = i;
                std::filesystem::path selectedPath = std::filesystem::path(state.targetDir) / state.offFiles[i];
                
                try {
                    pmp::SurfaceMesh tempMesh;
                    print::info("Reading mesh from: " + selectedPath.string());
                    pmp::read(tempMesh, selectedPath);

                    // Properties for ipg
                    auto exactPoints = tempMesh.add_vertex_property<ExactPoint>("v:exact_pos");
                    auto exactPlanes = tempMesh.add_face_property<ExactPlane>("f:exact_plane");

                    // Scale and snap vertex positions to integer grid and store in pmp property
                    for (auto v : tempMesh.vertices()) {
                        pmp::Point p = tempMesh.position(v);
                        tg::ipos3 ipos(
                            static_cast<int64_t>(p[0] * globalSettings::scaleFactor),
                            static_cast<int64_t>(p[1] * globalSettings::scaleFactor),
                            static_cast<int64_t>(p[2] * globalSettings::scaleFactor)
                        );
                        exactPoints[v] = ExactPoint(ipos);
                    }

                    for (auto face : tempMesh.faces()) {
                        auto it = tempMesh.vertices(face).begin();
                        ExactPoint vA = exactPoints[*it]; ++it;
                        ExactPoint vB = exactPoints[*it]; ++it;
                        ExactPoint vC = exactPoints[*it];

                        tg::pos<3, ExactGeom::pos_scalar_t> pA(int64_t(vA.x), int64_t(vA.y), int64_t(vA.z));
                        tg::pos<3, ExactGeom::pos_scalar_t> pB(int64_t(vB.x), int64_t(vB.y), int64_t(vB.z));
                        tg::pos<3, ExactGeom::pos_scalar_t> pC(int64_t(vC.x), int64_t(vC.y), int64_t(vC.z));

                        exactPlanes[face] = ExactPlane::from_points(pA, pB, pC);
                    }

                    state.mesh = std::move(tempMesh);
                    print::info("Mesh loaded: " + std::to_string(state.mesh.n_vertices()) + " vertices, " + std::to_string(state.mesh.n_faces()) + " faces.");

                    state.oSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::mesh), state.mesh);
                    state.oSMesh->setSurfaceColor(constants::colors::mesh);
                    state.pc = mesh_utils::register_pmp_pc(std::string(constants::polyNames::pc), state.mesh);
                    if (!state.mesh.is_empty()) state.meshLoaded = true;

                    mesh_utils::register_bbox(state);

                    // Point cloud cosmetics
                    state.pc->setEnabled(false);

                    // Reset camera to fit the new mesh
                    polyscope::view::resetCameraToHomeView();
                } catch (const std::exception& e) {
                    print::error("Error while loading " + selectedPath.string() + ": " + std::string(e.what()));
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
        if (ImGui::Button("Identify Concave Faces")) {
            std::vector<bool> isConcave = mesh_utils::identify_concave_faces(state.mesh);
            std::vector<double> scalarVal(state.mesh.n_faces());
            for (size_t i = 0; i < isConcave.size(); ++i) {
                scalarVal[i] = isConcave[i] ? 1.0 : 0.0;
            }
            state.oSMesh->addFaceScalarQuantity("isConcave", scalarVal)->setEnabled(true);
        }

        if (ImGui::Button("Generate Kernel")) {
            generate_kernel(state);
        }

        ImGui::Separator();
        ImGui::Text("Debug Kernel Generation");

        ImGui::Checkbox("Update Visuals During Stepping", &state.updateVisuals);

        if (!state.isSteppingKernel) {
            if (ImGui::Button("Start Kernel Stepping")) {
                init_kernel_stepping(state);
            }
        } else {
            ImGui::Text("Step: %d / %zu", state.currentPlaneIdx, state.supportPlanes.size());
            if (ImGui::Button("Next Step")) {
                step_kernel(state, state.updateVisuals);
            }
            ImGui::SameLine();
            if (ImGui::Button("Finish Kernel")) {
                while (state.isSteppingKernel) {
                    step_kernel(state, state.updateVisuals);
                }

                // Final visual update
                if (state.kSMesh) polyscope::removeStructure(state.kSMesh);
                state.kSMesh = mesh_utils::register_pmp_mesh(std::string(constants::polyNames::kernel), state.kHat);
                state.kSMesh->setSurfaceColor(constants::colors::kernel);
                state.kSMesh->setTransparency(constants::transparencies::kernel);
            }
            if (ImGui::Button("Cancel Stepping")) {
                state.isSteppingKernel = false;
            }
        }
        
    }
}

} // namespace gui
